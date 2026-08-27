#pragma once

#include "esphome/core/component.h"
#include "esphome/core/automation.h"
#include "esphome/components/binary_sensor/binary_sensor.h"
#include "esphome/components/button/button.h"
#include "esphome/components/cover/cover.h"
#include "esphome/components/sensor/sensor.h"

namespace esphome {
namespace sc_cover {

enum CoverTargetOperation : uint8_t {
  // order matters to match CoverOperation enum

  // stop door
  TARGET_OPERATION_IDLE = 0,
  // open door
  TARGET_OPERATION_OPEN,
  // close door
  TARGET_OPERATION_CLOSE,
  // do nothing (no action)
  TARGET_OPERATION_NONE,
  // activate door switch once
  TARGET_OPERATION_ACTIVATE_ONCE,
};

struct SingleControlCoverRestoreState {
  float position;
  uint8_t last_operation;
} __attribute__((packed));

class SingleControlCover : public cover::Cover, public Component {
 public:
  void setup() override;
  void loop() override;
  void dump_config() override;
  float get_setup_priority() const override;

  void set_door_activate_button(button::Button *door_activate_button) { this->door_activate_button_ = door_activate_button; }
  void set_button_press_interval(uint32_t button_press_interval) { this->button_press_interval_ = button_press_interval; }
  void set_setup_delay(uint32_t setup_delay) { this->setup_delay_ = setup_delay; }
  void set_operation_timeout(uint32_t operation_timeout) { this->operation_timeout_ = operation_timeout; }
  void set_motor_power_sensor(sensor::Sensor *motor_power_sensor) { this->motor_power_sensor_ = motor_power_sensor; }
  void set_motor_running_threshold(float threshold) { this->motor_running_threshold_ = threshold; }
  void set_motor_stopped_threshold(float threshold) { this->motor_stopped_threshold_ = threshold; }
  void set_motor_opening_max_power(float power) { this->motor_opening_max_power_ = power; }
  void set_motor_closing_min_power(float power) { this->motor_closing_min_power_ = power; }
  void set_open_endstop(binary_sensor::BinarySensor *open_endstop) { this->open_endstop_ = open_endstop; }
  void set_close_endstop(binary_sensor::BinarySensor *close_endstop) { this->close_endstop_ = close_endstop; }
  void set_open_duration(uint32_t open_duration) { this->open_duration_ = open_duration; }
  void set_close_duration(uint32_t close_duration) { this->close_duration_ = close_duration; }

  cover::CoverTraits get_traits() override;

 protected:
  void do_setup_();
  bool sync_from_endstops_();
  void finish_operation_(bool save_position);
  void save_state_();
  void control(const cover::CoverCall &call) override;
  bool is_open_() const { return this->open_endstop_->state; }
  bool is_closed_() const { return this->close_endstop_->state; }
  bool is_at_target_() const;
  bool is_operation_done_() const;

  void recompute_position_(const uint32_t now);

  bool activate_door_();

  void open_endstop_callback_(bool state);
  void close_endstop_callback_(bool state);
  void motor_power_callback_(float power);
  uint32_t estimate_transition_time_(uint32_t now) const;
  void apply_delayed_transition_(uint32_t now, uint32_t transitioned_at, cover::CoverOperation next_operation);
  void start_power_detected_operation_(uint32_t started_at, float power);
  void change_power_detected_direction_(cover::CoverOperation operation, uint32_t changed_at, float power);

  button::Button *door_activate_button_;
  binary_sensor::BinarySensor *open_endstop_;
  binary_sensor::BinarySensor *close_endstop_;
  sensor::Sensor *motor_power_sensor_{nullptr};
  bool toggle_{false};
  uint32_t button_press_interval_;
  uint32_t open_duration_;
  uint32_t close_duration_;
  uint32_t setup_delay_{0};
  uint32_t operation_timeout_{0};
  float motor_running_threshold_{100.0f};
  float motor_stopped_threshold_{50.0f};
  float motor_opening_max_power_{380.0f};
  float motor_closing_min_power_{390.0f};
  bool motor_power_initialized_{false};
  bool motor_running_{false};
  uint32_t last_motor_sample_time_{0};

  uint32_t last_activation_time_{0};
  uint32_t last_recompute_time_{0};
  uint32_t last_publish_time_{0};
  uint32_t operation_started_time_{0};
  float target_position_{0};
  cover::CoverOperation last_operation_{cover::COVER_OPERATION_OPENING};
  CoverTargetOperation target_operation_{TARGET_OPERATION_NONE};
  ESPPreferenceObject state_pref_;
};

}  // namespace sc_cover
}  // namespace esphome
