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

#include "twc_frames.h"

namespace esphome {
namespace twc_gateway {
namespace proto {

namespace {

void put_u16(std::vector<uint8_t> &out, uint16_t value) {
  out.push_back(static_cast<uint8_t>(value >> 8));
  out.push_back(static_cast<uint8_t>(value & 0xFF));
}

void put_zeros(std::vector<uint8_t> &out, size_t count) { out.insert(out.end(), count, 0x00); }

uint16_t get_u16(const uint8_t *data) { return static_cast<uint16_t>((data[0] << 8) | data[1]); }

uint32_t get_u32(const uint8_t *data) {
  return (static_cast<uint32_t>(data[0]) << 24) | (static_cast<uint32_t>(data[1]) << 16) |
         (static_cast<uint32_t>(data[2]) << 8) | static_cast<uint32_t>(data[3]);
}

std::vector<uint8_t> header(uint16_t command, uint16_t primary_id) {
  std::vector<uint8_t> out;
  out.reserve(16);
  put_u16(out, command);
  put_u16(out, primary_id);
  return out;
}

}  // namespace

uint8_t checksum(const uint8_t *data, size_t length) {
  uint8_t sum = 0;
  for (size_t i = 1; i < length; i++)
    sum += data[i];
  return sum;
}

std::vector<uint8_t> encode_frame(const std::vector<uint8_t> &payload) {
  if (payload.size() < 2)
    return {};
  uint16_t command = get_u16(payload.data());
  if (command == WRITE_ID_DATE || command == WRITE_MODEL_NO)
    return {};

  std::vector<uint8_t> out;
  out.reserve(payload.size() * 2 + 4);
  out.push_back(SLIP_END);
  auto put_escaped = [&out](uint8_t byte) {
    if (byte == SLIP_END) {
      out.push_back(SLIP_ESC);
      out.push_back(SLIP_ESC_END);
    } else if (byte == SLIP_ESC) {
      out.push_back(SLIP_ESC);
      out.push_back(SLIP_ESC_ESC);
    } else {
      out.push_back(byte);
    }
  };
  for (uint8_t byte : payload)
    put_escaped(byte);
  put_escaped(checksum(payload.data(), payload.size()));
  out.push_back(SLIP_END);
  // Real TWCs follow every frame with a trailing 0xFF (often seen as 0xFC-0xFE
  // after a few dropped bits); mirror that.
  out.push_back(0xFF);
  return out;
}

bool SlipDecoder::feed(uint8_t byte) {
  if (byte == SLIP_END) {
    if (!this->in_frame_ || this->index_ <= 2) {
      // Either the start of a frame, or an END immediately followed by
      // another END - which happens when we started listening mid-frame and
      // took the previous frame's END for a start. Treat it as a start.
      this->in_frame_ = true;
      this->reset_();
      return false;
    }
    size_t length = this->index_;
    this->in_frame_ = false;
    this->reset_();
    if (checksum(this->buffer_, length - 1) != this->buffer_[length - 1]) {
      this->checksum_errors_++;
      return false;
    }
    this->length_ = length;
    return true;
  }

  if (!this->in_frame_)
    return false;  // noise between frames, e.g. the trailing 0xFF

  if (this->escape_) {
    this->escape_ = false;
    if (byte == SLIP_ESC_END) {
      byte = SLIP_END;
    } else if (byte == SLIP_ESC_ESC) {
      byte = SLIP_ESC;
    } else {
      // Invalid escape sequence - drop the whole frame.
      this->in_frame_ = false;
      this->reset_();
      return false;
    }
  } else if (byte == SLIP_ESC) {
    this->escape_ = true;
    return false;
  }

  if (this->index_ >= MAX_FRAME_LENGTH) {
    this->overruns_++;
    this->in_frame_ = false;
    this->reset_();
    return false;
  }
  this->buffer_[this->index_++] = byte;
  return false;
}

std::vector<uint8_t> encode_linkready1(uint16_t primary_id, uint8_t sign) {
  auto out = header(PRIMARY_LINKREADY1, primary_id);
  out.push_back(sign);
  put_zeros(out, 8);
  return out;
}

std::vector<uint8_t> encode_linkready2(uint16_t primary_id, uint8_t sign) {
  auto out = header(PRIMARY_LINKREADY2, primary_id);
  out.push_back(sign);
  put_zeros(out, 8);
  return out;
}

std::vector<uint8_t> encode_heartbeat(uint16_t primary_id, uint16_t secondary_id, uint8_t command,
                                      uint16_t centiamps, uint8_t protocol) {
  auto out = header(PRIMARY_HEARTBEAT, primary_id);
  put_u16(out, secondary_id);
  out.push_back(command);
  put_u16(out, centiamps);
  put_zeros(out, protocol == 1 ? 4 : 6);  // 7 or 9 data bytes in total
  return out;
}

std::vector<uint8_t> encode_query(uint16_t command, uint16_t primary_id, uint16_t secondary_id) {
  auto out = header(command, primary_id);
  put_u16(out, secondary_id);
  put_zeros(out, 8);
  return out;
}

std::vector<uint8_t> encode_start_stop(bool start, uint16_t primary_id, uint16_t secondary_id) {
  auto out = header(start ? START_CHARGING : STOP_CHARGING, primary_id);
  put_u16(out, secondary_id);
  put_zeros(out, 9);
  return out;
}

Message decode(const uint8_t *frame, size_t length) {
  Message msg;
  // Every frame has at least a command, a sender ID and a checksum.
  if (length < 5) {
    msg.type = MessageType::INVALID;
    return msg;
  }
  msg.command = get_u16(frame);
  msg.src = get_u16(frame + 2);
  // Everything below indexes relative to `d` (first byte after the sender
  // ID); `n` is the number of data bytes before the checksum.
  const uint8_t *d = frame + 4;
  const size_t n = length - 5;

  auto require = [&msg, n](size_t needed, MessageType type) {
    msg.type = n >= needed ? type : MessageType::INVALID;
    return n >= needed;
  };

  switch (msg.command) {
    case SECONDARY_LINKREADY:
      if (require(3, MessageType::SECONDARY_LINKREADY)) {
        msg.sign = d[0];
        msg.max_allowable_centiamps = get_u16(d + 1);
      }
      break;

    case SECONDARY_HEARTBEAT:
      // dst(2) + state(1) + max(2) + actual(2) + 2 (protocol 1) or 4
      // (protocol 2) reserved bytes. TWCManager tells the two protocol
      // versions apart by exactly this length.
      if (require(7, MessageType::SECONDARY_HEARTBEAT)) {
        msg.dst = get_u16(d);
        msg.state = d[2];
        msg.reported_max_centiamps = get_u16(d + 3);
        msg.actual_centiamps = get_u16(d + 5);
        msg.protocol = n >= 11 ? 2 : 1;
      }
      break;

    case RESP_PWR_STATUS:
      if (require(13, MessageType::POWER_STATUS)) {
        msg.total_kwh = get_u32(d);
        for (int i = 0; i < 3; i++) {
          msg.voltage[i] = get_u16(d + 4 + 2 * i);
          msg.half_amps[i] = d[10 + i];
        }
      }
      break;

    case RESP_PLUG_STATE:
      if (require(1, MessageType::PLUG_STATE))
        msg.plug_state = d[0];
      break;

    case RESP_FIRMWARE_VER_EXT:
      if (require(4, MessageType::FIRMWARE)) {
        for (int i = 0; i < 4; i++)
          msg.firmware[i] = d[i];
      }
      break;

    case RESP_SERIAL_NUMBER:
      if (require(1, MessageType::SERIAL_NUMBER))
        msg.text.assign(reinterpret_cast<const char *>(d), n < 11 ? n : 11);
      break;

    case RESP_VIN_FIRST:
    case RESP_VIN_MIDDLE:
    case RESP_VIN_LAST: {
      size_t chars = msg.command == RESP_VIN_LAST ? 3 : 7;
      if (require(chars, MessageType::VIN_PART)) {
        msg.vin_part = msg.command == RESP_VIN_FIRST ? 0 : (msg.command == RESP_VIN_MIDDLE ? 1 : 2);
        msg.text.assign(reinterpret_cast<const char *>(d), chars);
      }
      break;
    }

    case PRIMARY_LINKREADY1:
    case PRIMARY_LINKREADY2:
      if (require(1, MessageType::PRIMARY_LINKREADY))
        msg.sign = d[0];
      break;

    case PRIMARY_HEARTBEAT:
      if (require(5, MessageType::PRIMARY_HEARTBEAT)) {
        msg.dst = get_u16(d);
        msg.state = d[2];
        msg.reported_max_centiamps = get_u16(d + 3);
      }
      break;

    default:
      msg.type = MessageType::UNKNOWN;
      break;
  }
  return msg;
}

bool sanitize_ascii(const std::string &raw, std::string &out) {
  out.clear();
  for (char c : raw) {
    auto byte = static_cast<uint8_t>(c);
    if (byte == 0)
      break;
    if (byte < 0x20 || byte > 0x7E)
      return false;
    out.push_back(c);
  }
  return true;
}

}  // namespace proto
}  // namespace twc_gateway
}  // namespace esphome
