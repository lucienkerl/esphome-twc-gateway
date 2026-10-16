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

#include "twc_gateway.h"

#include <cmath>

#include "esphome/core/helpers.h"
#include "esphome/core/log.h"

namespace esphome {
namespace twc_gateway {

static const char *const TAG = "twc-gateway";

// Measurements are published at most this often (heartbeats arrive ~1/s).
static constexpr uint32_t PUBLISH_INTERVAL_MS = 1000;
// Persist the energy meter every 0.1 kWh (~10 flash writes per kWh charged).
static constexpr double ENERGY_SAVE_STEP_KWH = 0.1;
static constexpr uint32_t WARN_INTERVAL_MS = 60000;

namespace {

// Every entity is optional in YAML; never touch one that wasn't configured.
// Also skips republishing an unchanged value.
void pub(sensor::Sensor *s, float value) {
  if (s == nullptr)
    return;
  if (s->has_state() && s->get_raw_state() == value)
    return;
  s->publish_state(value);
}

void pub(binary_sensor::BinarySensor *s, bool value) {
  if (s != nullptr)
    s->publish_state(value);  // BinarySensor already deduplicates
}

void pub(text_sensor::TextSensor *s, const std::string &value) {
  if (s == nullptr)
    return;
  if (s->has_state() && s->get_raw_state() == value)
    return;
  s->publish_state(value);
}

}  // namespace

void TWCGateway::setup() {
  if (this->flow_control_pin_ != nullptr) {
    this->flow_control_pin_->setup();
    this->flow_control_pin_->digital_write(false);
  }

  this->master_.configure(this->master_config_);
  this->apply_policy_();
  this->master_.set_send_callback([this](const std::vector<uint8_t> &frame) { this->transmit_(frame); });
  this->master_.set_event_callback(
      [this](MasterEvent event, const SecondaryInfo &info, uint16_t other_id) { this->on_event_(event, info, other_id); });

  // Boot values don't count as commands - the setpoint watchdog still
  // expects a real controller to show up within setpoint_timeout.
  this->master_.policy().set_initial_current(this->initial_current_);
  this->publish_state(this->master_.policy().requested_current());

  // AllowChargingSwitch is a plain switch_::Switch, not a Component, so
  // nothing resolves its restore mode for it - do that here, otherwise its
  // flash persistence never initialises (Switch::publish_state() only saves
  // once get_initial_state() has obtained its rtc_ object).
  if (this->allow_charging_switch_ != nullptr) {
    bool initial = this->allow_charging_switch_->get_initial_state_with_restore_mode().value_or(true);
    this->allow_charging_switch_->publish_state(initial);
    this->master_.policy().set_initial_enabled(initial);
  }

  // The meter reading is also what the Loxone bridge pushes as "Mr", so it
  // is persisted whether or not the energy sensor is configured - after a
  // reboot it must not fall back to the TWC's whole-kWh counter.
  this->energy_pref_ = global_preferences->make_preference<double>(fnv1_hash("twc-gateway-energy"));
  double saved;
  if (this->energy_pref_.load(&saved) && saved >= 0) {
    this->master_.energy().restore(saved);
    this->energy_saved_ = saved;
  }

  pub(this->link_binary_sensor_, false);
  this->publish_derived_();
}

void TWCGateway::loop() {
  uint32_t now = millis();
  uint8_t byte;
  while (this->available() && this->read_byte(&byte)) {
    if (this->decoder_.feed(byte))
      this->master_.on_frame(this->decoder_.frame(), this->decoder_.frame_length(), now);
  }
  if (this->decoder_.checksum_errors() != this->last_checksum_errors_) {
    this->last_checksum_errors_ = this->decoder_.checksum_errors();
    ESP_LOGD(TAG, "Frame with bad checksum dropped (%u so far) - check wiring/termination if this repeats",
             (unsigned) this->last_checksum_errors_);
  }

  this->master_.tick(now);

  if (this->measurements_dirty_ && now - this->last_publish_ms_ >= PUBLISH_INTERVAL_MS) {
    this->last_publish_ms_ = now;
    this->measurements_dirty_ = false;
    this->publish_measurements_();
    this->publish_derived_();
    this->save_energy_();
  }
  pub(this->failsafe_binary_sensor_, this->master_.failsafe_active(now));
}

void TWCGateway::transmit_(const std::vector<uint8_t> &frame) {
  if (this->flow_control_pin_ != nullptr)
    this->flow_control_pin_->digital_write(true);
  this->write_array(frame.data(), frame.size());
  // Wait until the last byte has left the UART before releasing the bus;
  // ~15 ms at 9600 baud for the longest frame.
  this->flush();
  if (this->flow_control_pin_ != nullptr)
    this->flow_control_pin_->digital_write(false);
}

void TWCGateway::on_event_(MasterEvent event, const SecondaryInfo &info, uint16_t other_id) {
  uint32_t now = millis();
  switch (event) {
    case MasterEvent::LINKED:
      ESP_LOGI(TAG, "Linked with TWC %04X (max %.2f A, sign %02X)", info.id, info.max_allowable_centiamps / 100.0f,
               info.sign);
      pub(this->link_binary_sensor_, true);
      if (info.max_allowable_centiamps > 0)
        pub(this->max_allowable_current_sensor_, info.max_allowable_centiamps / 100.0f);
      this->measurements_dirty_ = true;
      break;

    case MasterEvent::LOST:
      ESP_LOGW(TAG, "Lost TWC %04X - no answer for %u s. Values reset until it links again.", other_id,
               (unsigned) (this->master_config_.secondary_timeout_ms / 1000));
      pub(this->link_binary_sensor_, false);
      // Everything below would otherwise keep showing the last value, e.g.
      // "charging at 16 A" on a wallbox that has lost power.
      this->publish_measurements_();
      this->publish_derived_();
      break;

    case MasterEvent::STATE_CHANGED:
      ESP_LOGI(TAG, "Vehicle state changed for %04X: %u", info.id, info.vehicle_state);
      pub(this->state_sensor_, info.vehicle_state);
      this->publish_derived_();
      break;

    case MasterEvent::PLUG_STATE_CHANGED:
      ESP_LOGI(TAG, "Plug state changed for %04X: %d", info.id, info.plug_state);
      this->publish_derived_();
      break;

    case MasterEvent::IDENTITY:
      this->publish_identity_();
      break;

    case MasterEvent::OTHER_PRIMARY_SEEN:
      if (!this->other_primary_warned_ || now - this->last_other_primary_warn_ms_ > WARN_INTERVAL_MS) {
        this->other_primary_warned_ = true;
        this->last_other_primary_warn_ms_ = now;
        ESP_LOGW(TAG, "Another primary (%04X) is on the bus. Set the TWC's rotary switch to F (secondary), "
                      "or use passive_mode: true to only listen.", other_id);
      }
      break;

    case MasterEvent::EXTRA_SECONDARY_SEEN:
      if (!this->extra_secondary_warned_ || now - this->last_extra_secondary_warn_ms_ > WARN_INTERVAL_MS) {
        this->extra_secondary_warned_ = true;
        this->last_extra_secondary_warn_ms_ = now;
        ESP_LOGW(TAG, "Ignoring a second TWC (%04X): only one wall connector per gateway is supported.", other_id);
      }
      break;

    case MasterEvent::POWER_STATUS:
    case MasterEvent::HEARTBEAT:
      this->measurements_dirty_ = true;
      break;
  }
}

void TWCGateway::publish_measurements_() {
  const auto &s = this->master_.secondary();
  float actual = s.actual_centiamps / 100.0f;
  pub(this->actual_current_sensor_, actual);
  pub(this->current_sensor_, actual);  // single TWC: total == this connector
  pub(this->offered_current_sensor_, this->master_.offered_current());
  pub(this->power_sensor_, this->power_kw());

  if (s.have_power_status || !s.linked) {
    pub(this->phase_1_voltage_sensor_, s.voltage[0]);
    pub(this->phase_2_voltage_sensor_, s.voltage[1]);
    pub(this->phase_3_voltage_sensor_, s.voltage[2]);
    pub(this->phase_1_current_sensor_, s.half_amps[0] / 2.0f);
    pub(this->phase_2_current_sensor_, s.half_amps[1] / 2.0f);
    pub(this->phase_3_current_sensor_, s.half_amps[2] / 2.0f);
  }
  if (s.have_power_status)
    pub(this->total_kwh_delivered_sensor_, s.total_kwh);
  if (this->master_.energy().valid())
    pub(this->energy_sensor_, this->master_.energy().kwh());
  if (!s.linked)
    pub(this->state_sensor_, 0);
}

void TWCGateway::publish_identity_() {
  const auto &s = this->master_.secondary();
  if (!s.serial.empty())
    pub(this->serial_text_sensor_, s.serial);
  if (!s.firmware.empty())
    pub(this->firmware_version_text_sensor_, s.firmware);
  if (!s.vin.empty())
    pub(this->connected_vin_text_sensor_, s.vin);
}

void TWCGateway::publish_derived_() {
  bool charging = this->master_.charging();
  bool connected = this->master_.vehicle_connected();
  pub(this->charging_active_binary_sensor_, charging);
  pub(this->vehicle_connected_binary_sensor_, connected);
  pub(this->charging_active_numeric_sensor_, charging ? 1.0f : 0.0f);
  pub(this->vehicle_connected_numeric_sensor_, connected ? 1.0f : 0.0f);
}

void TWCGateway::save_energy_() {
  if (!this->master_.energy().valid())
    return;
  double kwh = this->master_.energy().kwh();
  if (this->energy_saved_ < 0 || std::fabs(kwh - this->energy_saved_) >= ENERGY_SAVE_STEP_KWH) {
    this->energy_pref_.save(&kwh);
    this->energy_saved_ = kwh;
  }
}

void TWCGateway::control(float value) {
  this->set_current_amps(static_cast<int>(std::lround(value)));
}

void TWCGateway::set_current_amps(int amps) {
  auto &policy = this->master_.policy();
  policy.set_requested_current(policy.clamp(amps), millis());
  // Publish what will actually be offered, not what was asked for: 3 A
  // becomes 6 A, anything above max_current becomes max_current.
  this->publish_state(policy.requested_current());
}

void TWCGateway::set_target_power_kw(float kw) { this->set_current_amps(this->master_.amps_for_power(kw)); }

void TWCGateway::set_charging_enabled(bool enabled) {
  if (enabled != this->master_.policy().enabled())
    ESP_LOGD(TAG, "Charging %s", enabled ? "allowed" : "not allowed");
  this->master_.policy().set_enabled(enabled, millis());
  // Re-send START/STOP_CHARGING even if unchanged, so a repeated
  // turn_on/turn_off works as a manual retry.
  this->master_.rearm_start_stop();
  if (this->allow_charging_switch_ != nullptr && this->allow_charging_switch_->state != enabled)
    this->allow_charging_switch_->publish_state(enabled);
}

void TWCGateway::dump_config() {
  const auto &pc = this->policy_config_;
  const auto &mc = this->master_config_;
  ESP_LOGCONFIG(TAG, "TWC Gateway:");
  ESP_LOGCONFIG(TAG, "  TWC ID: %04X%s", mc.twc_id, mc.passive ? " (passive, never transmits)" : "");
  ESP_LOGCONFIG(TAG, "  Current: %u-%u A, failsafe %u A", pc.min_current, pc.max_current, pc.failsafe_current);
  if (pc.setpoint_timeout_ms > 0) {
    ESP_LOGCONFIG(TAG, "  Setpoint watchdog: %u s", (unsigned) (pc.setpoint_timeout_ms / 1000));
  } else {
    ESP_LOGCONFIG(TAG, "  Setpoint watchdog: disabled");
  }
  ESP_LOGCONFIG(TAG, "  Min on/off time: %u s / %u s", (unsigned) (pc.min_on_time_ms / 1000),
                (unsigned) (pc.min_off_time_ms / 1000));
  ESP_LOGCONFIG(TAG, "  Start/stop commands: %s", YESNO(mc.use_start_stop_commands));
  ESP_LOGCONFIG(TAG, "  Phases (assumed): %u at %.0f V", mc.phases, mc.nominal_voltage);
  LOG_PIN("  Flow Control Pin: ", this->flow_control_pin_);
  LOG_NUMBER("  ", "Set Current", this);
}

}  // namespace twc_gateway
}  // namespace esphome
