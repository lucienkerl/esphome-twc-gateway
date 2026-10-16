/*
TWC Gateway for ESP32
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

#include "twc_policy.h"
#include "twc_frames.h"

namespace esphome {
namespace twc_gateway {

// A car told to charge at N A typically draws a bit less; TWCManager uses
// "> 4 A" as "the car has actually started taking power".
static constexpr uint16_t DRAWING_THRESHOLD_CENTIAMPS = 400;

void ChargePolicy::set_requested_current(uint8_t amps, uint32_t now_ms) {
  this->requested_ = this->clamp(amps);
  this->feed_watchdog(now_ms);
}

void ChargePolicy::set_enabled(bool enabled, uint32_t now_ms) {
  this->enabled_ = enabled;
  this->feed_watchdog(now_ms);
}

void ChargePolicy::feed_watchdog(uint32_t now_ms) {
  this->last_feed_ms_ = now_ms;
  this->fed_ = true;
}

uint8_t ChargePolicy::clamp(int amps) const {
  if (amps <= 0)
    return 0;
  if (amps > this->config_.max_current)
    return this->config_.max_current;
  if (amps < this->config_.min_current)
    return this->config_.min_current;
  return static_cast<uint8_t>(amps);
}

bool ChargePolicy::watchdog_expired(uint32_t now_ms) const {
  if (this->config_.setpoint_timeout_ms == 0)
    return false;
  // Before the first command ever arrives, count from boot (now_ms is
  // millis()), so a controller that never shows up also ends in failsafe.
  uint32_t since = this->fed_ ? now_ms - this->last_feed_ms_ : now_ms;
  return since > this->config_.setpoint_timeout_ms;
}

uint8_t ChargePolicy::effective_current(uint32_t now_ms, const SecondaryReport &report) {
  uint8_t target;
  if (this->watchdog_expired(now_ms)) {
    target = this->clamp(this->config_.failsafe_current);
  } else {
    // Clamp again: max_current can be lowered at runtime (e.g. from the web UI).
    target = this->enabled_ ? this->clamp(this->requested_) : 0;
  }

  // Minimum on time: once a car is actually drawing, don't yank the power
  // again within min_on_time - protects the TWC's relay and avoids putting
  // the car into a state where it won't resume without a replug.
  if (target == 0 && this->last_offered_ > 0 && this->config_.min_on_time_ms > 0 &&
      report.actual_centiamps > DRAWING_THRESHOLD_CENTIAMPS &&
      now_ms - this->offer_started_ms_ < this->config_.min_on_time_ms) {
    target = this->config_.min_current;
  }

  // Minimum off time: after stopping, wait before offering current again
  // (e.g. a PV surplus hovering around the minimum charging power).
  if (target > 0 && this->last_offered_ == 0 && this->ever_stopped_ && this->config_.min_off_time_ms > 0 &&
      now_ms - this->offer_stopped_ms_ < this->config_.min_off_time_ms) {
    target = 0;
  }

  if (target > 0 && this->last_offered_ == 0) {
    this->offer_started_ms_ = now_ms;
  } else if (target == 0 && this->last_offered_ > 0) {
    this->offer_stopped_ms_ = now_ms;
    this->ever_stopped_ = true;
  }
  this->last_offered_ = target;
  return target;
}

HeartbeatCommand ChargePolicy::heartbeat_command(uint8_t amps, const SecondaryReport &report) const {
  HeartbeatCommand hb;
  uint16_t centiamps = static_cast<uint16_t>(amps) * 100;
  if (amps == 0) {
    hb.command = proto::HB_SET_LIMIT_V1;
    hb.centiamps = 0;
    return hb;
  }
  if (this->config_.reassert_every_heartbeat || report.reported_max_centiamps != centiamps ||
      report.state == proto::ST_STARTING) {
    hb.command = report.protocol == 1 ? proto::HB_SET_LIMIT_V1 : proto::HB_SET_LIMIT_V2;
    hb.centiamps = centiamps;
    return hb;
  }
  hb.command = proto::HB_NO_CHANGE;
  return hb;
}

}  // namespace twc_gateway
}  // namespace esphome
