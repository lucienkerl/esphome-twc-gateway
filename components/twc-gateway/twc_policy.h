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

// Decides how much current to offer the secondary and which heartbeat
// command carries it. Pure C++ (no ESPHome includes) so it can be unit-
// tested on a host; all time is passed in as milliseconds.

#pragma once

#include <cstdint>

namespace esphome {
namespace twc_gateway {

struct PolicyConfig {
  uint8_t min_current{6};      // lowest non-zero current the protocol accepts
  uint8_t max_current{32};     // hard ceiling, protects the wiring
  uint8_t failsafe_current{0}; // offered once the setpoint watchdog expires
  uint32_t setpoint_timeout_ms{0};  // 0 = watchdog disabled
  uint32_t min_on_time_ms{0};   // keep offering current at least this long once a car draws
  uint32_t min_off_time_ms{0};  // stay at 0 A at least this long before offering again
  bool reassert_every_heartbeat{false};  // legacy: send the limit in every heartbeat
};

/// What the secondary currently reports about itself.
struct SecondaryReport {
  uint8_t protocol{2};
  uint8_t state{0};
  uint16_t reported_max_centiamps{0};
  uint16_t actual_centiamps{0};
};

struct HeartbeatCommand {
  uint8_t command{0};
  uint16_t centiamps{0};
};

class ChargePolicy {
 public:
  void configure(const PolicyConfig &config) { this->config_ = config; }
  const PolicyConfig &config() const { return this->config_; }

  /// Requested current in amps; clamped. Counts as a command (feeds the
  /// setpoint watchdog).
  void set_requested_current(uint8_t amps, uint32_t now_ms);
  /// Boot-time value; clamped, but does NOT feed the watchdog.
  void set_initial_current(uint8_t amps) { this->requested_ = this->clamp(amps); }
  /// Boot-time enable state; does NOT feed the watchdog.
  void set_initial_enabled(bool enabled) { this->enabled_ = enabled; }
  /// Allow Charging switch / Loxone "Ca". false offers 0 A.
  void set_enabled(bool enabled, uint32_t now_ms);
  /// Resets the setpoint watchdog without changing anything else.
  void feed_watchdog(uint32_t now_ms);

  /// 0 stays 0, 1..min-1 is raised to min, everything above max is capped.
  uint8_t clamp(int amps) const;

  /// Current that should be offered right now (after watchdog, enable,
  /// minimum on/off time). Call once per heartbeat.
  uint8_t effective_current(uint32_t now_ms, const SecondaryReport &report);

  /// Heartbeat command for the given effective current. Mirrors
  /// TWCManager's _amps_heartbeat(): 0 A always goes out as 0x05 (no relay
  /// chatter); a non-zero limit is (re)sent until the secondary echoes it in
  /// its own heartbeat, and while it sits in state 08; otherwise 0x00.
  HeartbeatCommand heartbeat_command(uint8_t amps, const SecondaryReport &report) const;

  bool watchdog_expired(uint32_t now_ms) const;
  bool enabled() const { return this->enabled_; }
  uint8_t requested_current() const { return this->requested_; }

 protected:
  PolicyConfig config_{};
  uint8_t requested_{0};
  bool enabled_{true};
  uint32_t last_feed_ms_{0};
  bool fed_{false};

  uint8_t last_offered_{0};
  uint32_t offer_started_ms_{0};   // when we went from 0 to >0
  uint32_t offer_stopped_ms_{0};   // when we went from >0 to 0
  bool ever_stopped_{false};
};

}  // namespace twc_gateway
}  // namespace esphome
