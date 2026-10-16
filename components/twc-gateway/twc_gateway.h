/*
TWC Gateway for ESP32
Copyright (C) 2023 Jarl Nicolson
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

#pragma once

#include "esphome/core/component.h"
#include "esphome/core/hal.h"
#include "esphome/core/preferences.h"
#include "esphome/components/binary_sensor/binary_sensor.h"
#include "esphome/components/number/number.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/text_sensor/text_sensor.h"
#include "esphome/components/uart/uart.h"

#include "twc_frames.h"
#include "twc_master.h"
#include "twc_switch.h"

namespace esphome {
namespace twc_gateway {

/// ESPHome glue around TwcMaster: feeds it UART bytes and millis(), puts
/// its frames on the RS-485 bus, and publishes what it learns. The entity
/// itself is the "Set Current" number.
class TWCGateway : public number::Number, public uart::UARTDevice, public Component {
 public:
  void setup() override;
  void loop() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::DATA; }

  // --- configuration (called from generated code, before setup()) --------
  void set_twcid(uint16_t id) { this->master_config_.twc_id = id; }
  void set_passive_mode(bool passive) { this->master_config_.passive = passive; }
  void set_flow_control_pin(GPIOPin *pin) { this->flow_control_pin_ = pin; }
  void set_query_vin(bool v) { this->master_config_.query_vin = v; }
  void set_use_start_stop_commands(bool v) { this->master_config_.use_start_stop_commands = v; }
  void set_phases(uint8_t phases) { this->master_config_.phases = phases; }
  void set_nominal_voltage(float v) { this->master_config_.nominal_voltage = v; }

  // Policy settings take effect immediately: a command can arrive over the
  // network before setup() has run.
  void set_min_current(uint8_t a) { this->policy_config_.min_current = a < 6 ? 6 : a; this->apply_policy_(); }
  void set_max_current(uint8_t a) { this->policy_config_.max_current = a; this->apply_policy_(); }
  void set_failsafe_current(uint8_t a) { this->policy_config_.failsafe_current = a; this->apply_policy_(); }
  void set_setpoint_timeout(uint32_t ms) { this->policy_config_.setpoint_timeout_ms = ms; this->apply_policy_(); }
  void set_min_on_time(uint32_t ms) { this->policy_config_.min_on_time_ms = ms; this->apply_policy_(); }
  void set_min_off_time(uint32_t ms) { this->policy_config_.min_off_time_ms = ms; this->apply_policy_(); }
  void set_reassert_current(bool v) { this->policy_config_.reassert_every_heartbeat = v; this->apply_policy_(); }
  void set_initial_current(uint8_t a) { this->initial_current_ = a; }
  void set_restore_current(bool v) { this->restore_current_ = v; }
  void set_allow_charging_switch(AllowChargingSwitch *sw) { this->allow_charging_switch_ = sw; }

  SUB_SENSOR(current)
  SUB_SENSOR(max_allowable_current)
  SUB_SENSOR(total_kwh_delivered)
  SUB_SENSOR(phase_1_voltage)
  SUB_SENSOR(phase_2_voltage)
  SUB_SENSOR(phase_3_voltage)
  SUB_SENSOR(phase_1_current)
  SUB_SENSOR(phase_2_current)
  SUB_SENSOR(phase_3_current)
  SUB_SENSOR(actual_current)
  SUB_SENSOR(state)
  SUB_SENSOR(power)
  SUB_SENSOR(energy)
  SUB_SENSOR(offered_current)
  // Numeric (0/1) mirrors of the binary sensors, for MQTT consumers that
  // want a number instead of ESPHome's fixed "ON"/"OFF" payload.
  SUB_SENSOR(vehicle_connected_numeric)
  SUB_SENSOR(charging_active_numeric)

  SUB_TEXT_SENSOR(serial)
  SUB_TEXT_SENSOR(firmware_version)
  SUB_TEXT_SENSOR(connected_vin)

  SUB_BINARY_SENSOR(vehicle_connected)
  SUB_BINARY_SENSOR(charging_active)
  SUB_BINARY_SENSOR(link)
  SUB_BINARY_SENSOR(failsafe)

  // --- control API (main loop only) ------------------------------------
  /// Allow Charging switch / Loxone "Ca".
  void set_charging_enabled(bool enabled);
  /// Current in amps; clamped like the number entity.
  void set_current_amps(int amps);
  /// Power target in kW (Loxone Wallbox block output "Tp").
  void set_target_power_kw(float kw);
  /// Count as "the controller is alive" without changing anything.
  void feed_watchdog() { this->master_.policy().feed_watchdog(millis()); }

  // --- state for other components (e.g. the Loxone bridge) --------------
  const TwcMaster &master() const { return this->master_; }
  bool is_vehicle_connected() const { return this->master_.vehicle_connected(); }
  bool is_charging() const { return this->master_.charging(); }
  float power_kw() const { return this->master_.power_w() / 1000.0f; }
  bool energy_valid() const { return this->master_.energy().valid(); }
  double energy_kwh() const { return this->master_.energy().kwh(); }
  bool failsafe_active() const { return this->master_.failsafe_active(millis()); }

 protected:
  void control(float value) override;
  void apply_policy_() { this->master_.policy().configure(this->policy_config_); }
  void transmit_(const std::vector<uint8_t> &frame);
  void on_event_(MasterEvent event, const SecondaryInfo &info, uint16_t other_id);
  void publish_measurements_();
  void publish_identity_();
  void publish_derived_();
  void save_energy_();

  TwcMaster master_{};
  proto::SlipDecoder decoder_{};
  MasterConfig master_config_{};
  PolicyConfig policy_config_{};
  GPIOPin *flow_control_pin_{nullptr};
  AllowChargingSwitch *allow_charging_switch_{nullptr};
  uint8_t initial_current_{0};
  bool restore_current_{false};
  ESPPreferenceObject current_pref_;
  uint8_t current_saved_{0xFF};

  uint32_t last_publish_ms_{0};
  bool measurements_dirty_{false};
  uint32_t last_other_primary_warn_ms_{0};
  uint32_t last_extra_secondary_warn_ms_{0};
  bool other_primary_warned_{false};
  bool extra_secondary_warned_{false};
  uint32_t last_checksum_errors_{0};

  ESPPreferenceObject energy_pref_;
  double energy_saved_{-1};
};

}  // namespace twc_gateway
}  // namespace esphome
