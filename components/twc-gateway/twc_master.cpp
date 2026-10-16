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

#include "twc_master.h"

#include <cmath>
#include <cstdio>
#include <cstring>

namespace esphome {
namespace twc_gateway {

namespace {

// Wrap-safe "now has reached due".
bool reached(uint32_t now_ms, uint32_t due_ms) { return static_cast<int32_t>(now_ms - due_ms) >= 0; }

// A phase counts as "in use" from 1 A (2 half-amp steps) up.
constexpr uint8_t ACTIVE_PHASE_HALF_AMPS = 2;
// Above this the car is drawing; some TWCs report ~0.25 A while idle.
constexpr uint16_t CHARGING_CENTIAMPS = 100;

constexpr uint32_t STARTUP_INTERVAL_MS = 1000;
constexpr uint8_t STARTUP_MESSAGES = 5;
// Above this, an echo-only heartbeat counts as "charging" (idle TWCs can
// report ~0.25 A).
constexpr uint16_t ECHO_CHARGING_CENTIAMPS = 100;
constexpr uint32_t PLUG_INTERVAL_MS = 3000;
constexpr uint32_t POWER_INTERVAL_MS = 5000;
constexpr uint32_t IDENTITY_RETRY_MS = 5000;
constexpr uint32_t VIN_INTERVAL_MS = 30000;
// Gaps longer than this (e.g. a lost link) are not integrated.
constexpr uint32_t MAX_INTEGRATION_GAP_MS = 60000;

}  // namespace

// --- EnergyMeter ---------------------------------------------------------------

void EnergyMeter::bound_() {
  // Never below what the TWC itself has counted, never a full kWh above it
  // - and never backwards, which would look like an invalid meter reading.
  double lower = this->counter_;
  double upper = this->counter_ + 0.999;
  if (this->value_ < lower)
    this->value_ = lower;
  if (this->value_ > upper)
    this->value_ = upper;
}

void EnergyMeter::on_counter(uint32_t twc_kwh) {
  if (!this->initialized_) {
    this->counter_ = twc_kwh;
    if (this->has_restored_ && this->restored_ >= twc_kwh && this->restored_ < twc_kwh + 1.0) {
      this->value_ = this->restored_;
    } else {
      this->value_ = twc_kwh;
    }
    this->initialized_ = true;
    return;
  }
  if (twc_kwh < this->counter_)
    return;  // a corrupt frame; the TWC's lifetime counter never decreases
  double previous = this->value_;
  this->counter_ = twc_kwh;
  this->bound_();
  if (this->value_ < previous)
    this->value_ = previous;
}

void EnergyMeter::on_power(double watts, uint32_t now_ms) {
  if (this->has_last_ && this->initialized_) {
    uint32_t dt = now_ms - this->last_ms_;
    if (dt <= MAX_INTEGRATION_GAP_MS) {
      double previous = this->value_;
      this->value_ += this->last_watts_ * (static_cast<double>(dt) / 3600000.0) / 1000.0;
      this->bound_();
      if (this->value_ < previous)
        this->value_ = previous;
    }
  }
  this->last_watts_ = watts;
  this->last_ms_ = now_ms;
  this->has_last_ = true;
}

// --- TwcMaster -----------------------------------------------------------------

void TwcMaster::send_(const std::vector<uint8_t> &payload) {
  if (this->config_.passive || !this->send_fn_)
    return;
  auto frame = proto::encode_frame(payload);
  if (!frame.empty())
    this->send_fn_(frame);
}

void TwcMaster::emit_(MasterEvent event, uint16_t other_id) {
  if (this->event_fn_)
    this->event_fn_(event, this->secondary_, other_id);
}

void TwcMaster::tick(uint32_t now_ms) {
  if (this->secondary_.linked && now_ms - this->secondary_.last_rx_ms > this->config_.secondary_timeout_ms) {
    this->lose_(now_ms);
  }

  if (this->config_.passive)
    return;

  switch (this->phase_) {
    case Phase::LINKREADY1:
    case Phase::LINKREADY2: {
      if (this->started_ && now_ms - this->last_startup_ms_ < STARTUP_INTERVAL_MS)
        return;
      this->started_ = true;
      this->last_startup_ms_ = now_ms;
      bool first = this->phase_ == Phase::LINKREADY1;
      this->send_(first ? proto::encode_linkready1(this->config_.twc_id, this->config_.sign)
                        : proto::encode_linkready2(this->config_.twc_id, this->config_.sign));
      if (++this->startup_count_ >= STARTUP_MESSAGES) {
        this->startup_count_ = 0;
        this->phase_ = first ? Phase::LINKREADY2 : Phase::RUNNING;
      }
      return;
    }
    case Phase::RUNNING:
      break;
  }

  if (!this->secondary_.linked)
    return;

  if (now_ms - this->last_heartbeat_ms_ >= this->config_.heartbeat_interval_ms) {
    this->send_heartbeat_(now_ms);
  } else if (this->aux_pending_ && reached(now_ms, this->aux_due_ms_)) {
    this->send_aux_(now_ms);
  }
}

void TwcMaster::send_heartbeat_(uint32_t now_ms) {
  const auto &s = this->secondary_;
  SecondaryReport report{s.protocol, s.state, s.reported_max_centiamps, s.actual_centiamps};
  uint8_t amps = this->policy_.effective_current(now_ms, report);
  // The TWC's own hardware maximum is a second ceiling on top of max_current.
  if (s.max_allowable_centiamps >= 600 && amps * 100 > s.max_allowable_centiamps)
    amps = s.max_allowable_centiamps / 100;
  this->offered_ = amps;

  HeartbeatCommand hb = this->policy_.heartbeat_command(amps, report);
  this->last_heartbeat_ = hb;
  this->send_(proto::encode_heartbeat(this->config_.twc_id, s.id, hb.command, hb.centiamps, s.protocol));

  this->last_heartbeat_ms_ = now_ms;
  this->aux_pending_ = true;
  this->aux_due_ms_ = now_ms + this->config_.heartbeat_interval_ms / 2;
}

void TwcMaster::send_aux_(uint32_t now_ms) {
  // At most one extra frame between two heartbeats, so the secondary always
  // has time to answer and the heartbeat cadence never slips.
  this->aux_pending_ = false;
  auto &s = this->secondary_;
  const uint16_t me = this->config_.twc_id;

  if (this->config_.use_start_stop_commands) {
    int8_t desired = this->offered_ > 0 ? 1 : 0;
    if (s.start_stop_last_sent != desired) {
      s.start_stop_last_sent = desired;
      this->send_(proto::encode_start_stop(desired == 1, me, s.id));
      this->emit_(MasterEvent::START_STOP_SENT, desired);
      return;
    }
  }

  if (reached(now_ms, this->next_plug_ms_)) {
    this->next_plug_ms_ = now_ms + PLUG_INTERVAL_MS;
    this->send_(proto::encode_query(proto::GET_PLUG_STATE, me, s.id));
    return;
  }
  if (reached(now_ms, this->next_power_ms_)) {
    this->next_power_ms_ = now_ms + POWER_INTERVAL_MS;
    this->send_(proto::encode_query(proto::GET_PWR_STATE, me, s.id));
    return;
  }
  if ((s.serial.empty() || s.firmware.empty()) && reached(now_ms, this->next_identity_ms_)) {
    this->next_identity_ms_ = now_ms + IDENTITY_RETRY_MS;
    this->send_(proto::encode_query(s.serial.empty() ? proto::GET_SERIAL_NUMBER : proto::GET_FIRMWARE_VER_EXT, me,
                                    s.id));
    return;
  }
  if (this->config_.query_vin && reached(now_ms, this->next_vin_ms_)) {
    static const uint16_t VIN_COMMANDS[3] = {proto::GET_VIN_FIRST, proto::GET_VIN_MIDDLE, proto::GET_VIN_LAST};
    this->send_(proto::encode_query(VIN_COMMANDS[this->vin_part_next_], me, s.id));
    this->vin_part_next_ = (this->vin_part_next_ + 1) % 3;
    this->next_vin_ms_ = this->vin_part_next_ == 0 ? now_ms + VIN_INTERVAL_MS : now_ms;
    return;
  }
}

void TwcMaster::link_(uint16_t id, uint8_t sign, uint16_t max_centiamps, uint32_t now_ms) {
  this->secondary_ = SecondaryInfo{};
  this->secondary_.linked = true;
  this->secondary_.id = id;
  this->secondary_.sign = sign;
  this->secondary_.max_allowable_centiamps = max_centiamps;
  this->secondary_.last_rx_ms = now_ms;
  this->next_plug_ms_ = now_ms;
  this->next_power_ms_ = now_ms;
  this->next_identity_ms_ = now_ms;
  this->next_vin_ms_ = now_ms;
  this->vin_part_next_ = 0;
  this->vin_parts_seen_ = 0;
  // Answer with a heartbeat as soon as the startup sequence allows.
  this->last_heartbeat_ms_ = now_ms - this->config_.heartbeat_interval_ms;
  this->aux_pending_ = false;
  this->emit_(MasterEvent::LINKED);
}

void TwcMaster::lose_(uint32_t now_ms) {
  (void) now_ms;
  uint16_t id = this->secondary_.id;
  this->secondary_ = SecondaryInfo{};
  this->offered_ = 0;
  this->last_heartbeat_ = HeartbeatCommand{};
  this->aux_pending_ = false;
  this->emit_(MasterEvent::LOST, id);
}

void TwcMaster::on_frame(const uint8_t *frame, size_t length, uint32_t now_ms) {
  proto::Message msg = proto::decode(frame, length);
  // On a transceiver that switches direction by itself we may hear our own
  // transmissions; they are not news.
  if (msg.src == this->config_.twc_id)
    return;

  switch (msg.type) {
    case proto::MessageType::UNKNOWN:
      // Only from our own secondary: anything the TWC says that we don't
      // understand may be exactly what explains a car that won't charge.
      if (this->secondary_.linked && msg.src == this->secondary_.id)
        this->emit_(MasterEvent::UNKNOWN_FRAME, msg.command);
      return;
    case proto::MessageType::INVALID:
      return;

    case proto::MessageType::PRIMARY_LINKREADY:
    case proto::MessageType::PRIMARY_HEARTBEAT:
      if (!this->config_.passive)
        this->emit_(MasterEvent::OTHER_PRIMARY_SEEN, msg.src);
      return;

    case proto::MessageType::SECONDARY_LINKREADY:
      if (!this->secondary_.linked) {
        this->link_(msg.src, msg.sign, msg.max_allowable_centiamps, now_ms);
      } else if (msg.src == this->secondary_.id) {
        // The secondary went back to advertising itself: it stopped hearing
        // us. Keep the link and answer right away.
        this->secondary_.last_rx_ms = now_ms;
        this->secondary_.max_allowable_centiamps = msg.max_allowable_centiamps;
        this->last_heartbeat_ms_ = now_ms - this->config_.heartbeat_interval_ms;
      } else {
        this->emit_(MasterEvent::EXTRA_SECONDARY_SEEN, msg.src);
      }
      return;

    default:
      break;
  }

  if (!this->secondary_.linked) {
    // In passive mode we never see a linkready addressed to us; adopt the
    // first secondary that reports in.
    if (this->config_.passive && (msg.type == proto::MessageType::SECONDARY_HEARTBEAT ||
                                  msg.type == proto::MessageType::POWER_STATUS)) {
      this->link_(msg.src, 0, 0, now_ms);
    } else {
      return;
    }
  }
  if (msg.src != this->secondary_.id) {
    if (msg.type == proto::MessageType::SECONDARY_HEARTBEAT)
      this->emit_(MasterEvent::EXTRA_SECONDARY_SEEN, msg.src);
    return;
  }
  this->handle_secondary_(msg, now_ms);
}

void TwcMaster::handle_secondary_(const proto::Message &msg, uint32_t now_ms) {
  auto &s = this->secondary_;
  s.last_rx_ms = now_ms;

  switch (msg.type) {
    case proto::MessageType::SECONDARY_HEARTBEAT: {
      s.protocol = msg.protocol;
      s.state = msg.state;
      s.reported_max_centiamps = msg.reported_max_centiamps;
      s.actual_centiamps = msg.actual_centiamps;
      // 0x09 is only the echo of our own limit command, not a vehicle
      // status; it interleaves with the real one and must not mask it.
      // With reassert_current every heartbeat is an echo and the real status
      // never comes through, so infer charging / not charging from the
      // actual current instead.
      uint8_t st = msg.state;
      if (st == proto::ST_LIMIT_ECHO) {
        if (msg.actual_centiamps > ECHO_CHARGING_CENTIAMPS) {
          st = proto::ST_CHARGING;
        } else if (s.vehicle_state == proto::ST_CHARGING) {
          st = proto::ST_PLUGGED_READY;
        }
      }
      if (st != proto::ST_LIMIT_ECHO && st != s.vehicle_state) {
        auto engaged = [](uint8_t st) {
          return st == proto::ST_CHARGING || st == proto::ST_PLUGGED_DO_NOT_CHARGE ||
                 st == proto::ST_PLUGGED_READY || st == proto::ST_STARTING;
        };
        // A new charging session (car plugged in / woke up): re-send the
        // start/stop command, it doesn't carry over from the last session.
        if (!engaged(s.vehicle_state) && engaged(st))
          s.start_stop_last_sent = -1;
        s.vehicle_state = st;
        this->emit_(MasterEvent::STATE_CHANGED);
      }
      this->energy_.on_power(this->power_w(), now_ms);
      this->emit_(MasterEvent::HEARTBEAT);
      break;
    }

    case proto::MessageType::POWER_STATUS:
      s.have_power_status = true;
      s.total_kwh = msg.total_kwh;
      for (int i = 0; i < 3; i++) {
        s.voltage[i] = msg.voltage[i];
        s.half_amps[i] = msg.half_amps[i];
      }
      this->energy_.on_counter(msg.total_kwh);
      this->emit_(MasterEvent::POWER_STATUS);
      break;

    case proto::MessageType::PLUG_STATE:
      if (s.plug_state != static_cast<int8_t>(msg.plug_state)) {
        s.plug_state = static_cast<int8_t>(msg.plug_state);
        this->emit_(MasterEvent::PLUG_STATE_CHANGED);
      }
      break;

    case proto::MessageType::FIRMWARE: {
      char buf[20];
      snprintf(buf, sizeof(buf), "%u.%u.%u.%u", msg.firmware[0], msg.firmware[1], msg.firmware[2], msg.firmware[3]);
      if (s.firmware != buf) {
        s.firmware = buf;
        this->emit_(MasterEvent::IDENTITY);
      }
      break;
    }

    case proto::MessageType::SERIAL_NUMBER: {
      std::string serial;
      if (proto::sanitize_ascii(msg.text, serial) && !serial.empty() && serial != s.serial) {
        s.serial = serial;
        this->emit_(MasterEvent::IDENTITY);
      }
      break;
    }

    case proto::MessageType::VIN_PART: {
      static const size_t OFFSETS[3] = {0, 7, 14};
      memcpy(s.vin_raw + OFFSETS[msg.vin_part], msg.text.data(), msg.text.size());
      this->vin_parts_seen_ |= 1 << msg.vin_part;
      if (this->vin_parts_seen_ != 0x07)
        break;
      std::string vin;
      if (!proto::sanitize_ascii(std::string(s.vin_raw, sizeof(s.vin_raw)), vin))
        break;  // corrupt
      if (vin.empty()) {
        vin = "0";
      } else if (vin.size() != 17) {
        break;  // partial or corrupt
      }
      if (vin != s.vin) {
        s.vin = vin;
        this->emit_(MasterEvent::IDENTITY);
      }
      break;
    }

    default:
      break;
  }
}

uint8_t TwcMaster::active_phases() const {
  const auto &s = this->secondary_;
  uint8_t count = 0;
  if (s.have_power_status) {
    for (int i = 0; i < 3; i++) {
      if (s.half_amps[i] >= ACTIVE_PHASE_HALF_AMPS)
        count++;
    }
  }
  return count > 0 ? count : this->config_.phases;
}

double TwcMaster::power_w() const {
  const auto &s = this->secondary_;
  if (!s.linked || s.actual_centiamps < CHARGING_CENTIAMPS)
    return 0.0;
  double amps = s.actual_centiamps / 100.0;
  double volts = 0;
  uint8_t phases = 0;
  if (s.have_power_status) {
    for (int i = 0; i < 3; i++) {
      if (s.half_amps[i] >= ACTIVE_PHASE_HALF_AMPS) {
        volts += s.voltage[i];
        phases++;
      }
    }
  }
  if (phases == 0) {
    // Not visible in the power status yet: assume the configured number of
    // phases at their measured (or nominal) voltage.
    double sum = 0;
    uint8_t n = 0;
    for (int i = 0; s.have_power_status && i < 3; i++) {
      if (s.voltage[i] > 100) {
        sum += s.voltage[i];
        n++;
      }
    }
    double per_phase = n > 0 ? sum / n : this->config_.nominal_voltage;
    volts = per_phase * this->config_.phases;
  }
  return amps * volts;
}

uint8_t TwcMaster::amps_for_power(double kw) const {
  if (!(kw > 0))
    return 0;
  // Nominal (not measured) voltage on purpose: it's what Loxone uses for its
  // own min/max charging power (6 A x 3 x 230 V = 4.14 kW), so its minimum
  // never lands just below 6 A because the grid happens to be at 236 V.
  double amps = kw * 1000.0 / (this->config_.nominal_voltage * this->active_phases());
  long rounded = std::lround(amps);
  if (rounded < this->policy_.config().min_current)
    return 0;
  return this->policy_.clamp(static_cast<int>(rounded));
}

bool TwcMaster::charging() const {
  const auto &s = this->secondary_;
  if (!s.linked)
    return false;
  return s.vehicle_state == proto::ST_CHARGING || s.vehicle_state == proto::ST_STARTING ||
         s.actual_centiamps >= CHARGING_CENTIAMPS;
}

bool TwcMaster::vehicle_connected() const {
  const auto &s = this->secondary_;
  if (!s.linked)
    return false;
  // GET_PLUG_STATE is unambiguous once answered; the heartbeat state alone
  // collapses "fully charged, still plugged in" into 0 = "maybe unplugged".
  if (s.plug_state >= 0)
    return s.plug_state != 0;
  return this->charging() || s.vehicle_state == proto::ST_PLUGGED_DO_NOT_CHARGE ||
         s.vehicle_state == proto::ST_PLUGGED_READY;
}

}  // namespace twc_gateway
}  // namespace esphome
