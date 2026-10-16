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

// The fake-primary ("fake master") state machine. Non-blocking: the owner
// calls tick() from its loop and on_frame() for every received frame, and
// gets transmissions back through the send callback. No threads, no delays,
// no ESPHome includes - see tests/ for host-side tests.
//
// Exactly one secondary TWC is supported: one TWC maps onto one Loxone
// Wallbox block, and sharing power between several wallboxes is better done
// by Loxone's Wallbox Manager than by this device.

#pragma once

#include <cstdint>
#include <functional>
#include <string>
#include <vector>

#include "twc_frames.h"
#include "twc_policy.h"

namespace esphome {
namespace twc_gateway {

struct MasterConfig {
  uint16_t twc_id{0xABCD};
  uint8_t sign{0x77};
  bool passive{false};
  bool query_vin{true};
  bool use_start_stop_commands{false};
  uint32_t heartbeat_interval_ms{1000};
  uint32_t secondary_timeout_ms{26000};  // a real primary gives up after ~26 s
  uint8_t phases{3};                     // assumed until the car shows how many it uses
  float nominal_voltage{230.0f};         // per phase, for kW <-> A conversion
};

struct SecondaryInfo {
  bool linked{false};
  uint16_t id{0};
  uint8_t sign{0};
  uint8_t protocol{2};
  uint16_t max_allowable_centiamps{0};
  uint32_t last_rx_ms{0};

  uint8_t state{0};          // raw heartbeat state, including the 0x09 echo
  uint8_t vehicle_state{0};  // last state that was not the 0x09 echo
  uint16_t reported_max_centiamps{0};
  uint16_t actual_centiamps{0};

  int8_t plug_state{-1};  // -1 = never answered GET_PLUG_STATE

  bool have_power_status{false};
  uint32_t total_kwh{0};
  uint16_t voltage[3]{0, 0, 0};
  uint8_t half_amps[3]{0, 0, 0};

  std::string serial;
  std::string firmware;
  std::string vin;  // "" until all three parts arrived; "0" = no car / none reported
  char vin_raw[17]{};

  int8_t start_stop_last_sent{-1};  // -1 never, 0 stop, 1 start
};

enum class MasterEvent : uint8_t {
  LINKED,
  LOST,
  STATE_CHANGED,         // vehicle_state changed (ignores the 0x09 echo)
  PLUG_STATE_CHANGED,
  POWER_STATUS,          // new voltages/currents/kWh
  IDENTITY,              // serial, firmware or VIN changed
  OTHER_PRIMARY_SEEN,    // someone else is acting as primary on this bus
  EXTRA_SECONDARY_SEEN,  // a second secondary tried to link - ignored
  HEARTBEAT,             // any secondary heartbeat (actual current etc.)
  START_STOP_SENT,       // a START/STOP_CHARGING frame went out (other_id: 1 start, 0 stop)
  UNKNOWN_FRAME,         // a valid frame we don't decode (other_id: its command)
};

/// Turns the TWC's coarse lifetime counter (whole kWh) into a monotonic
/// meter reading with Wh resolution by integrating power between counter
/// steps, while never drifting more than one kWh away from the TWC itself.
class EnergyMeter {
 public:
  /// Value saved before a reboot; used only if it matches the TWC's counter.
  void restore(double kwh) {
    this->restored_ = kwh;
    this->has_restored_ = true;
  }
  void on_counter(uint32_t twc_kwh);
  void on_power(double watts, uint32_t now_ms);
  bool valid() const { return this->initialized_; }
  double kwh() const { return this->value_; }

 protected:
  void bound_();
  double value_{0};
  double restored_{0};
  bool has_restored_{false};
  bool initialized_{false};
  uint32_t counter_{0};
  double last_watts_{0};
  uint32_t last_ms_{0};
  bool has_last_{false};
};

class TwcMaster {
 public:
  using SendFn = std::function<void(const std::vector<uint8_t> &frame)>;
  using EventFn = std::function<void(MasterEvent event, const SecondaryInfo &info, uint16_t other_id)>;

  void configure(const MasterConfig &config) { this->config_ = config; }
  const MasterConfig &config() const { return this->config_; }
  void set_send_callback(SendFn fn) { this->send_fn_ = std::move(fn); }
  void set_event_callback(EventFn fn) { this->event_fn_ = std::move(fn); }

  ChargePolicy &policy() { return this->policy_; }
  const ChargePolicy &policy() const { return this->policy_; }
  const SecondaryInfo &secondary() const { return this->secondary_; }

  /// Drive timers; call frequently (every loop iteration is fine).
  void tick(uint32_t now_ms);
  /// One checksum-verified frame from the bus (SlipDecoder output).
  void on_frame(const uint8_t *frame, size_t length, uint32_t now_ms);

  /// Re-send START/STOP_CHARGING on the next slot even if unchanged (lets a
  /// repeated turn_on/turn_off act as a manual retry).
  void rearm_start_stop() { this->secondary_.start_stop_last_sent = -1; }

  /// Charging power in W: the heartbeat's actual current (1 s updates,
  /// 0.01 A resolution) times the voltages of the phases the power status
  /// shows as carrying current.
  double power_w() const;
  /// Number of phases the car is currently drawing on (configured value if
  /// it isn't drawing yet).
  uint8_t active_phases() const;
  /// Convert a power target (Loxone Wallbox block output "Tp", kW) into the
  /// current to offer: floor(P / (U * phases)), 0 below min_current.
  uint8_t amps_for_power(double kw) const;

  bool charging() const;
  bool vehicle_connected() const;

  uint8_t offered_current() const { return this->offered_; }
  /// The heartbeat command and limit most recently sent to the secondary.
  const HeartbeatCommand &last_heartbeat() const { return this->last_heartbeat_; }
  bool failsafe_active(uint32_t now_ms) const { return this->policy_.watchdog_expired(now_ms); }
  EnergyMeter &energy() { return this->energy_; }
  const EnergyMeter &energy() const { return this->energy_; }

 protected:
  enum class Phase : uint8_t { LINKREADY1, LINKREADY2, RUNNING };

  void send_(const std::vector<uint8_t> &payload);
  void emit_(MasterEvent event, uint16_t other_id = 0);
  void link_(uint16_t id, uint8_t sign, uint16_t max_centiamps, uint32_t now_ms);
  void lose_(uint32_t now_ms);
  void send_heartbeat_(uint32_t now_ms);
  void send_aux_(uint32_t now_ms);
  void handle_secondary_(const proto::Message &msg, uint32_t now_ms);

  MasterConfig config_{};
  ChargePolicy policy_{};
  EnergyMeter energy_{};
  SendFn send_fn_{};
  EventFn event_fn_{};
  SecondaryInfo secondary_{};

  Phase phase_{Phase::LINKREADY1};
  uint8_t startup_count_{0};
  uint32_t last_startup_ms_{0};
  bool started_{false};

  uint32_t last_heartbeat_ms_{0};
  bool aux_pending_{false};
  uint32_t aux_due_ms_{0};
  uint8_t offered_{0};
  HeartbeatCommand last_heartbeat_{};

  uint32_t next_plug_ms_{0};
  uint32_t next_power_ms_{0};
  uint32_t next_identity_ms_{0};
  uint32_t next_vin_ms_{0};
  uint8_t vin_part_next_{0};
  uint8_t vin_parts_seen_{0};
};

}  // namespace twc_gateway
}  // namespace esphome
