/*
TWC Gateway for ESP32
Copyright (C) 2020 Craig Peacock
Copyright (C) 2020 Jarl Nicolson
This program is free software; you can redistribute it and/or
modify it under the terms of the GNU General Public License
as published by the Free Software Foundation; either version 3
of the License, or (at your option) any later version.
This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.
You should have received a copy of the GNU General Public License
along with this program.  If not, see <https://www.gnu.org/licenses/>.
*/

// Wire format of the Tesla Wall Connector Gen 2 load-sharing protocol:
// SLIP framing, checksums, and encoding/decoding of every frame this
// project sends or understands.
//
// This file is deliberately free of any ESPHome/Arduino/ESP-IDF include so
// it can be compiled and unit-tested on a host machine (see tests/).
//
// Frame layouts follow TWCManager (ngardiner/TWCManager), the most
// actively maintained reverse-engineered implementation:
//   - lib/TWCManager/TWCSlave.py  (heartbeat semantics, state codes)
//   - lib/TWCManager/TWCMaster.py (linkready, queries, start/stop lengths)

#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace esphome {
namespace twc_gateway {
namespace proto {

// --- Command words (first two bytes of every frame) ------------------------

// Primary (master) -> secondary (slave), expects a response
static constexpr uint16_t GET_PLUG_STATE = 0xFBB4;
static constexpr uint16_t PRIMARY_HEARTBEAT = 0xFBE0;
static constexpr uint16_t PRIMARY_LINKREADY2 = 0xFBE2;
static constexpr uint16_t GET_PWR_STATE = 0xFBEB;
static constexpr uint16_t GET_FIRMWARE_VER_EXT = 0xFBEC;
static constexpr uint16_t GET_SERIAL_NUMBER = 0xFBED;
static constexpr uint16_t GET_VIN_FIRST = 0xFBEE;
static constexpr uint16_t GET_VIN_MIDDLE = 0xFBEF;
static constexpr uint16_t GET_VIN_LAST = 0xFBF1;

// Primary -> secondary, no response
static constexpr uint16_t START_CHARGING = 0xFCB1;
static constexpr uint16_t STOP_CHARGING = 0xFCB2;
static constexpr uint16_t PRIMARY_LINKREADY1 = 0xFCE1;

// ** DANGEROUS ** - permanently rewrite a TWC's identity. Defined only so
// encode_frame() can refuse to ever put them on the wire.
static constexpr uint16_t WRITE_ID_DATE = 0xFC19;
static constexpr uint16_t WRITE_MODEL_NO = 0xFC1A;

// Secondary -> primary
static constexpr uint16_t RESP_PLUG_STATE = 0xFDB4;
static constexpr uint16_t SECONDARY_HEARTBEAT = 0xFDE0;
static constexpr uint16_t SECONDARY_LINKREADY = 0xFDE2;
static constexpr uint16_t RESP_PWR_STATUS = 0xFDEB;
static constexpr uint16_t RESP_FIRMWARE_VER_EXT = 0xFDEC;
static constexpr uint16_t RESP_SERIAL_NUMBER = 0xFDED;
static constexpr uint16_t RESP_VIN_FIRST = 0xFDEE;
static constexpr uint16_t RESP_VIN_MIDDLE = 0xFDEF;
static constexpr uint16_t RESP_VIN_LAST = 0xFDF1;

// --- Heartbeat command byte (primary -> secondary, byte 0 of the data) -----
static constexpr uint8_t HB_NO_CHANGE = 0x00;
static constexpr uint8_t HB_ERROR = 0x02;  // never sent by this project
// Protocol 1: set limit. Protocol 2: set limit *before* a car charges; also
// what TWCManager sends to offer 0 A, because 0x09 + 0 A makes the secondary
// bounce between states 01 and 03 and click its relay every second.
static constexpr uint8_t HB_SET_LIMIT_V1 = 0x05;
static constexpr uint8_t HB_SET_LIMIT_V2 = 0x09;

// --- Secondary heartbeat state byte ----------------------------------------
static constexpr uint8_t ST_READY = 0x00;  // car may or may not be plugged in
static constexpr uint8_t ST_CHARGING = 0x01;
static constexpr uint8_t ST_ERROR = 0x02;
static constexpr uint8_t ST_PLUGGED_DO_NOT_CHARGE = 0x03;
static constexpr uint8_t ST_PLUGGED_READY = 0x04;
static constexpr uint8_t ST_BUSY = 0x05;
static constexpr uint8_t ST_STARTING = 0x08;
static constexpr uint8_t ST_LIMIT_ECHO = 0x09;  // echo of our own 0x09 command
static constexpr uint8_t ST_ADJUSTMENT_COMPLETE = 0x0A;

// --- SLIP --------------------------------------------------------------------
static constexpr uint8_t SLIP_END = 0xC0;
static constexpr uint8_t SLIP_ESC = 0xDB;
static constexpr uint8_t SLIP_ESC_END = 0xDC;
static constexpr uint8_t SLIP_ESC_ESC = 0xDD;

static constexpr size_t MAX_FRAME_LENGTH = 32;

/// Sum of every byte except the first (the protocol's checksum definition).
uint8_t checksum(const uint8_t *data, size_t length);

/// Appends the checksum, SLIP-escapes and frames `payload` (C0 ... C0 FF).
/// Returns an empty vector for the blocked WRITE_* commands.
std::vector<uint8_t> encode_frame(const std::vector<uint8_t> &payload);

/// Byte-at-a-time SLIP decoder. Never blocks, keeps escape state between
/// calls, and drops anything malformed instead of passing it on.
class SlipDecoder {
 public:
  /// Feed one byte. Returns true when a complete, checksum-valid frame is
  /// available through frame()/frame_length() (valid until the next feed()).
  bool feed(uint8_t byte);
  const uint8_t *frame() const { return this->buffer_; }
  size_t frame_length() const { return this->length_; }

  uint32_t checksum_errors() const { return this->checksum_errors_; }
  uint32_t overruns() const { return this->overruns_; }

 protected:
  void reset_() {
    this->index_ = 0;
    this->escape_ = false;
  }

  uint8_t buffer_[MAX_FRAME_LENGTH]{};
  size_t index_{0};
  size_t length_{0};
  bool in_frame_{false};
  bool escape_{false};
  uint32_t checksum_errors_{0};
  uint32_t overruns_{0};
};

// --- Encoders (return the unframed payload; pass it to encode_frame()) -----

std::vector<uint8_t> encode_linkready1(uint16_t primary_id, uint8_t sign);
std::vector<uint8_t> encode_linkready2(uint16_t primary_id, uint8_t sign);
/// `protocol` is 1 (7 data bytes) or 2 (9 data bytes).
std::vector<uint8_t> encode_heartbeat(uint16_t primary_id, uint16_t secondary_id, uint8_t command,
                                      uint16_t centiamps, uint8_t protocol);
/// Queries (GET_*): 8 trailing zero bytes, as TWCManager sends them.
std::vector<uint8_t> encode_query(uint16_t command, uint16_t primary_id, uint16_t secondary_id);
/// START_CHARGING / STOP_CHARGING: 9 trailing zero bytes (TWCManager's
/// sendStartCommand()/sendStopCommand()), one more than a query.
std::vector<uint8_t> encode_start_stop(bool start, uint16_t primary_id, uint16_t secondary_id);

// --- Decoded messages --------------------------------------------------------

enum class MessageType : uint8_t {
  UNKNOWN,
  INVALID,  // known command, but too short to decode
  SECONDARY_LINKREADY,
  SECONDARY_HEARTBEAT,
  POWER_STATUS,
  PLUG_STATE,
  FIRMWARE,
  SERIAL_NUMBER,
  VIN_PART,
  PRIMARY_LINKREADY,  // another master on the bus
  PRIMARY_HEARTBEAT,  // another master on the bus (or ours echoed)
};

struct Message {
  MessageType type{MessageType::UNKNOWN};
  uint16_t command{0};
  uint16_t src{0};
  uint16_t dst{0};

  // SECONDARY_LINKREADY
  uint8_t sign{0};
  uint16_t max_allowable_centiamps{0};

  // SECONDARY_HEARTBEAT
  uint8_t protocol{2};
  uint8_t state{0};
  uint16_t reported_max_centiamps{0};
  uint16_t actual_centiamps{0};

  // POWER_STATUS
  uint32_t total_kwh{0};
  uint16_t voltage[3]{0, 0, 0};
  uint8_t half_amps[3]{0, 0, 0};  // per-phase current in 0.5 A steps

  // PLUG_STATE: 0=unplugged, 1=charging, 2=unknown, 3=plugged, not charging
  uint8_t plug_state{0};

  // FIRMWARE
  uint8_t firmware[4]{0, 0, 0, 0};

  // SERIAL_NUMBER / VIN_PART: raw bytes, not yet validated as ASCII
  uint8_t vin_part{0};  // 0=first, 1=middle, 2=last
  std::string text;
};

/// Decodes one checksum-verified frame (as returned by SlipDecoder, with the
/// trailing checksum byte still attached). Length is checked against the
/// fields that are read, so a short frame can never yield garbage values.
Message decode(const uint8_t *frame, size_t length);

/// Printable-ASCII filter for serial numbers and VINs. Returns false if any
/// byte before the first NUL is non-printable; a corrupt frame must never
/// reach the API as invalid UTF-8 (aioesphomeapi drops the connection).
bool sanitize_ascii(const std::string &raw, std::string &out);

}  // namespace proto
}  // namespace twc_gateway
}  // namespace esphome
