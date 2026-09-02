#include "sc_cover.h"
#include "esphome/core/log.h"
#include "esphome/core/hal.h"

#include <algorithm>
#include <cmath>

namespace esphome {
namespace sc_cover {

static const char *const TAG = "sc_cover.cover";
static const uint32_t RESTORE_STATE_VERSION = 0x53434301;
static const uint32_t CLOSING_STOP_MARGIN_MS = 200;

using namespace esphome::cover;

CoverTraits SingleControlCover::get_traits() {
  auto traits = CoverTraits();
  traits.set_supports_stop(true);
  traits.set_supports_position(true);
  traits.set_supports_toggle(true);
  traits.set_is_assumed_state(true);
  traits.set_supports_tilt(false);

  return traits;
}

void SingleControlCover::dump_config() {
  LOG_COVER("", "SingleControl Cover", this);
  LOG_BUTTON(" ", "Door Switch", this->door_activate_button_);
  ESP_LOGCONFIG(TAG, " Setup delay: %.1fs", this->setup_delay_ /1e3f);
  ESP_LOGCONFIG(TAG, " Switch Interval:  %.1fs", this->button_press_interval_ / 1e3f);
  ESP_LOGCONFIG(TAG, " Closing Stop Delay: %.1fs", this->effective_closing_stop_delay_() / 1e3f);
  LOG_BINARY_SENSOR("  ", "Open Endstop", this->open_endstop_);
  ESP_LOGCONFIG(TAG, "  Open Duration: %.1fs", this->open_duration_ / 1e3f);
  LOG_BINARY_SENSOR("  ", "Close Endstop", this->close_endstop_);
  ESP_LOGCONFIG(TAG, "  Close Duration: %.1fs", this->close_duration_ / 1e3f);
  if (this->operation_timeout_ > 0)
    ESP_LOGCONFIG(TAG, " Operation Timeout: %.1fs", this->operation_timeout_ / 1e3f);
  if (this->motor_power_sensor_ != nullptr) {
    LOG_SENSOR("  ", "Motor Power Sensor", this->motor_power_sensor_);
    ESP_LOGCONFIG(TAG, " Motor Running Threshold: %.1f W", this->motor_running_threshold_);
    ESP_LOGCONFIG(TAG, " Motor Stopped Threshold: %.1f W", this->motor_stopped_threshold_);
    ESP_LOGCONFIG(TAG, " Motor Opening Maximum: %.1f W", this->motor_opening_max_power_);
    ESP_LOGCONFIG(TAG, " Motor Closing Minimum: %.1f W", this->motor_closing_min_power_);
  }
}

float SingleControlCover::get_setup_priority() const { return setup_priority::WIFI; }

void SingleControlCover::setup() {
  this->open_endstop_->add_on_state_callback([this](bool state) { this->open_endstop_callback_(state); });
  this->close_endstop_->add_on_state_callback([this](bool state) { this->close_endstop_callback_(state); });
  if (this->motor_power_sensor_ != nullptr) {
    this->motor_power_sensor_->add_on_state_callback([this](float power) { this->motor_power_callback_(power); });
  }

  this->state_pref_ = this->make_entity_preference<SingleControlCoverRestoreState>(RESTORE_STATE_VERSION);
  SingleControlCoverRestoreState restore{0.5f, static_cast<uint8_t>(COVER_OPERATION_OPENING)};
  if (this->state_pref_.load(&restore)) {
    this->position = clamp(restore.position, 0.0f, 1.0f);
    if (restore.last_operation == COVER_OPERATION_OPENING || restore.last_operation == COVER_OPERATION_CLOSING)
      this->last_operation_ = static_cast<CoverOperation>(restore.last_operation);
  } else {
    this->position = 0.5f;
  }
  this->current_operation = COVER_OPERATION_IDLE;
  this->target_position_ = this->position;
  this->last_recompute_time_ = millis();
  this->do_setup_();

  if (this->setup_delay_ > 0) {
    this->set_timeout(this->setup_delay_, [this] { this->do_setup_(); });
  }
}

void SingleControlCover::do_setup_() {
  if (!this->sync_from_endstops_()) {
    if (this->position == COVER_CLOSED)
      this->position = 0.01f;
    else if (this->position == COVER_OPEN)
      this->position = 0.99f;
  }
  this->target_position_ = this->position;
  this->publish_state(false);
}

bool SingleControlCover::sync_from_endstops_() {
  if (this->open_endstop_->has_state() && this->is_open_()) {
    this->position = COVER_OPEN;
    this->last_operation_ = COVER_OPERATION_OPENING;
    this->finish_operation_(true);
    return true;
  }
  if (this->close_endstop_->has_state() && this->is_closed_()) {
    this->position = COVER_CLOSED;
    this->last_operation_ = COVER_OPERATION_CLOSING;
    this->finish_operation_(true);
    return true;
  }
  return false;
}

void SingleControlCover::finish_operation_(bool save_position) {
  this->current_operation = COVER_OPERATION_IDLE;
  this->target_operation_ = TARGET_OPERATION_NONE;
  this->target_position_ = this->position;
  this->operation_started_time_ = 0;
  this->publish_state(false);
  if (save_position)
    this->save_state_();
  this->last_publish_time_ = millis();
}

void SingleControlCover::save_state_() {
  SingleControlCoverRestoreState state{this->position, static_cast<uint8_t>(this->last_operation_)};
  this->state_pref_.save(&state);
}

void SingleControlCover::control(const CoverCall &call) {
  // this function will be called every time the user requests a state change.

  if (call.get_stop()) {
    // requested to stop

    ESP_LOGD(TAG, "Stop command received.");

    if (this->current_operation != COVER_OPERATION_IDLE) {
      this->target_operation_ = TARGET_OPERATION_IDLE;
    }
  }

  if (call.get_toggle().has_value()) {
    this->target_operation_ = TARGET_OPERATION_NONE;
    this->toggle_ = true;
  }

  if (call.get_position().has_value()) {
    // requested to change position

    // get requested position
    auto pos = *call.get_position();

    // ensure position is between 0 and 1
    pos = clamp(pos, 0.0f, 1.0f);

    ESP_LOGD(TAG, "Position command received: %0.2f", pos);

    if (pos != this->position) {
      // not at target -> calculate target operation

      this->target_operation_ = pos < this->position ? TARGET_OPERATION_CLOSE : TARGET_OPERATION_OPEN;
      this->target_position_ = pos;
    }
  }
}

void SingleControlCover::loop() {
  // This will be called by App.loop()

  // store current time
  const uint32_t now = millis();

  // recompute position every loop cycle
  this->recompute_position_(now);

  if (this->current_operation != COVER_OPERATION_IDLE && this->target_operation_ == TARGET_OPERATION_NONE &&
      !this->toggle_ && this->operation_timeout_ > 0 &&
      this->operation_started_time_ != 0 && (now - this->operation_started_time_) >= this->operation_timeout_) {
    ESP_LOGW(TAG, "Movement timed out without reaching an endstop; keeping the estimated position");
    this->finish_operation_(true);
    return;
  }

  if (this->toggle_) {
    // toggle requested
    if (this->activate_door_()) {
      this->toggle_ = false;
      if (this->current_operation == COVER_OPERATION_CLOSING) {
        this->target_position_ = COVER_CLOSED;
      } else if (this->current_operation == COVER_OPERATION_OPENING) {
        this->target_position_ = COVER_OPEN;
      } else {
        this->target_position_ = this->position;
      }
    }
  }

  else if ((this->target_operation_ != TARGET_OPERATION_NONE) && !this->is_operation_done_()) {
    // target operation not done -> activate switch
    if (this->target_operation_ == TARGET_OPERATION_IDLE)
      this->stop_door_();
    else
      this->activate_door_();
  }

  else if (this->is_operation_done_()) {
    // target operation done -> clear operation
    ESP_LOGD(TAG, "Target operation reached");
    this->target_operation_ = TARGET_OPERATION_NONE;
  }

  else if ((this->current_operation != COVER_OPERATION_IDLE) && this->is_at_target_()) {
    // target operation was done, door is moving and target position reached

    // let door stop by itself if FULL_OPEN or FULL_CLOSE requested
    if (this->target_position_ != COVER_CLOSED && this->target_position_ != COVER_OPEN) {
      this->stop_door_();
    }
  }

  // send current position every second
  if (this->current_operation != COVER_OPERATION_IDLE && (now - this->last_publish_time_) > 1000) {
    this->publish_state(false);
    this->last_publish_time_ = now;
  }
}

bool SingleControlCover::is_at_target_() const {
  if (this->current_operation == COVER_OPERATION_OPENING)
    return this->position >= this->target_position_;

  if (this->current_operation == COVER_OPERATION_CLOSING) {
    float stop_position = this->target_position_;
    if (stop_position != COVER_CLOSED && stop_position != COVER_OPEN) {
      const float reverse_travel = static_cast<float>(this->effective_closing_stop_delay_()) / this->open_duration_;
      stop_position = std::max(0.01f, stop_position - reverse_travel);
    }
    return this->position <= stop_position;
  }

  return false;
}

uint32_t SingleControlCover::effective_closing_stop_delay_() const {
  return std::max(this->closing_stop_delay_, this->button_press_interval_ + CLOSING_STOP_MARGIN_MS);
}

bool SingleControlCover::is_operation_done_() const {
  return static_cast<uint8_t>(this->target_operation_) == static_cast<uint8_t>(this->current_operation);
}

void SingleControlCover::recompute_position_(const uint32_t now) {
  // only recompute position if door is moving
  if (this->current_operation != COVER_OPERATION_IDLE) {
    float dir;
    float action_dur;

    // set dir and duration depending on current movement
    if (this->current_operation == COVER_OPERATION_CLOSING) {
      // door closing
      dir = -1.0f;
      action_dur = this->close_duration_;
    } else {
      // door opening
      dir = 1.0f;
      action_dur = this->open_duration_;
    }

    // calculate position
    float change = (dir * (now - this->last_recompute_time_)) / action_dur;
    const float minimum = this->is_closed_() ? COVER_CLOSED : 0.01f;
    const float maximum = this->is_open_() ? COVER_OPEN : 0.99f;
    this->position = clamp(this->position + change, minimum, maximum);
  }

  // store time
  this->last_recompute_time_ = now;
}

bool SingleControlCover::activate_door_() {
  // store current time
  const uint32_t now = millis();

  if ((now - this->last_activation_time_) > this->button_press_interval_) {
    // cover state machine (recompute current operation)
    bool stopped = false;
    if (this->current_operation == COVER_OPERATION_OPENING) {
      // This opener stops when its button is pressed while opening.
      this->current_operation = COVER_OPERATION_IDLE;
      this->operation_started_time_ = 0;
      stopped = true;
    } else if (this->current_operation == COVER_OPERATION_CLOSING) {
      // This opener immediately reverses when its button is pressed while closing.
      this->current_operation = COVER_OPERATION_OPENING;
      this->last_operation_ = COVER_OPERATION_OPENING;
      this->target_position_ = COVER_OPEN;
      this->target_operation_ = TARGET_OPERATION_NONE;
      this->operation_started_time_ = now;
    } else {
      // door idle, check last direction
      if (this->last_operation_ == COVER_OPERATION_OPENING) {
        // last operation: opening -> new operation: closing
        this->current_operation = COVER_OPERATION_CLOSING;
        this->last_operation_ = COVER_OPERATION_CLOSING;
      } else {
        // last operation: closing -> new operation: opening
        this->current_operation = COVER_OPERATION_OPENING;
        this->last_operation_ = COVER_OPERATION_OPENING;
      }
    }

    // activate switch
    ESP_LOGD(TAG, "Switch activated");
    this->door_activate_button_->press();

    if (!stopped && this->operation_started_time_ == 0)
      this->operation_started_time_ = now;

    // send current state
    this->publish_state(false);
    if (stopped)
      this->save_state_();
    this->last_publish_time_ = now;
    this->last_recompute_time_ = now;
    this->last_activation_time_ = now;

    // return switch activated
    return true;
  }

  // return switch not activated
  return false;
}

bool SingleControlCover::stop_door_() {
  if (this->current_operation == COVER_OPERATION_IDLE || this->closing_stop_pending_)
    return false;

  if (this->current_operation != COVER_OPERATION_CLOSING)
    return this->activate_door_();

  const uint32_t now = millis();
  if ((now - this->last_activation_time_) <= this->button_press_interval_)
    return false;

  // This opener reverses on the first press while closing. A second press is
  // required to stop the resulting opening movement.
  ESP_LOGI(TAG, "Stopping closing movement with two button presses");
  this->closing_stop_pending_ = true;
  this->target_operation_ = TARGET_OPERATION_NONE;
  this->current_operation = COVER_OPERATION_OPENING;
  this->last_operation_ = COVER_OPERATION_OPENING;
  this->operation_started_time_ = now;
  this->door_activate_button_->press();
  this->publish_state(false);
  this->last_publish_time_ = now;
  this->last_recompute_time_ = now;
  this->last_activation_time_ = now;

  const uint32_t second_press_delay = this->effective_closing_stop_delay_();
  ESP_LOGI(TAG, "Reversing now; stop press scheduled in %.1fs", second_press_delay / 1e3f);
  this->set_timeout("closing-stop-second-press", second_press_delay, [this] {
    if (!this->closing_stop_pending_)
      return;

    const uint32_t stopped_at = millis();
    this->recompute_position_(stopped_at);
    ESP_LOGI(TAG, "Sending second button press to stop after closing reversal");
    this->door_activate_button_->press();
    this->closing_stop_pending_ = false;
    this->last_activation_time_ = stopped_at;
    this->last_recompute_time_ = stopped_at;
    this->finish_operation_(true);
  });
  return true;
}

void SingleControlCover::open_endstop_callback_(bool state) {
  if (state) {
    // open end stop reached
    this->closing_stop_pending_ = false;
    this->cancel_timeout("closing-stop-second-press");
    this->last_activation_time_ = millis();
    this->last_recompute_time_ = this->last_activation_time_;
    this->current_operation = COVER_OPERATION_IDLE;
    this->last_operation_ = COVER_OPERATION_OPENING;
    this->position = COVER_OPEN;
    this->target_position_ = COVER_OPEN;
    this->operation_started_time_ = 0;
    this->publish_state(false);
    this->save_state_();
  } else {
    // open end stop leaving
    if (this->current_operation != COVER_OPERATION_CLOSING) {
      // external close commanded (like external remote)
      this->current_operation = COVER_OPERATION_CLOSING;
      this->last_operation_ = COVER_OPERATION_CLOSING;
      this->target_position_ = COVER_CLOSED;
      this->last_activation_time_ = millis();
      this->last_recompute_time_ = this->last_activation_time_;
      this->operation_started_time_ = this->last_activation_time_;
      this->publish_state(false);
      this->last_publish_time_ = this->last_activation_time_;
    }
  }
}

void SingleControlCover::close_endstop_callback_(bool state) {
  if (state) {
    // close end stop reached
    this->closing_stop_pending_ = false;
    this->cancel_timeout("closing-stop-second-press");
    this->last_activation_time_ = millis();
    this->last_recompute_time_ = this->last_activation_time_;
    this->current_operation = COVER_OPERATION_IDLE;
    this->last_operation_ = COVER_OPERATION_CLOSING;
    this->position = COVER_CLOSED;
    this->target_position_ = COVER_CLOSED;
    this->operation_started_time_ = 0;
    this->publish_state(false);
    this->save_state_();
  } else {
    // close end stop leaving
    if (this->current_operation != COVER_OPERATION_OPENING) {
      // external open commanded (like external remote)
      this->current_operation = COVER_OPERATION_OPENING;
      this->last_operation_ = COVER_OPERATION_OPENING;
      this->target_position_ = COVER_OPEN;
      this->last_activation_time_ = millis();
      this->last_recompute_time_ = this->last_activation_time_;
      this->operation_started_time_ = this->last_activation_time_;
      this->publish_state(false);
      this->last_publish_time_ = this->last_activation_time_;
    }
  }
}

uint32_t SingleControlCover::estimate_transition_time_(uint32_t now) const {
  if (this->last_motor_sample_time_ == 0)
    return now;

  const uint32_t sample_interval = now - this->last_motor_sample_time_;
  return now - std::min(sample_interval / 2, static_cast<uint32_t>(2000));
}

void SingleControlCover::apply_delayed_transition_(uint32_t now, uint32_t transitioned_at,
                                                   CoverOperation next_operation) {
  this->recompute_position_(now);
  if (this->current_operation == COVER_OPERATION_IDLE || transitioned_at == now)
    return;

  const float old_rate = this->current_operation == COVER_OPERATION_CLOSING ? -1.0f / this->close_duration_
                                                                            : 1.0f / this->open_duration_;
  float new_rate = 0.0f;
  if (next_operation == COVER_OPERATION_CLOSING)
    new_rate = -1.0f / this->close_duration_;
  else if (next_operation == COVER_OPERATION_OPENING)
    new_rate = 1.0f / this->open_duration_;

  const float correction = (new_rate - old_rate) * (now - transitioned_at);
  const float minimum = this->is_closed_() ? COVER_CLOSED : 0.01f;
  const float maximum = this->is_open_() ? COVER_OPEN : 0.99f;
  this->position = clamp(this->position + correction, minimum, maximum);
  this->last_recompute_time_ = now;
}

void SingleControlCover::start_power_detected_operation_(uint32_t started_at, float power) {
  if (this->current_operation != COVER_OPERATION_IDLE)
    return;

  if (this->is_open_()) {
    this->current_operation = COVER_OPERATION_CLOSING;
  } else if (this->is_closed_()) {
    this->current_operation = COVER_OPERATION_OPENING;
  } else if (power >= this->motor_closing_min_power_) {
    this->current_operation = COVER_OPERATION_CLOSING;
  } else if (power <= this->motor_opening_max_power_) {
    this->current_operation = COVER_OPERATION_OPENING;
  } else if (this->last_operation_ == COVER_OPERATION_OPENING) {
    this->current_operation = COVER_OPERATION_CLOSING;
  } else {
    this->current_operation = COVER_OPERATION_OPENING;
  }

  ESP_LOGI(TAG, "Power-detected movement started %s at %.1f W",
           this->current_operation == COVER_OPERATION_OPENING ? "opening" : "closing", power);
  this->last_operation_ = this->current_operation;
  this->target_position_ = this->current_operation == COVER_OPERATION_CLOSING ? COVER_CLOSED : COVER_OPEN;
  this->target_operation_ = TARGET_OPERATION_NONE;
  this->operation_started_time_ = started_at;
  this->last_recompute_time_ = started_at;
  this->last_publish_time_ = millis();
  this->publish_state(false);
}

void SingleControlCover::change_power_detected_direction_(CoverOperation operation, uint32_t changed_at, float power) {
  if (this->current_operation == operation)
    return;

  const uint32_t now = millis();
  this->apply_delayed_transition_(now, changed_at, operation);
  ESP_LOGI(TAG, "Motor direction changed to %s at %.1f W", operation == COVER_OPERATION_OPENING ? "opening" : "closing",
           power);
  this->current_operation = operation;
  this->last_operation_ = operation;
  this->target_position_ = operation == COVER_OPERATION_CLOSING ? COVER_CLOSED : COVER_OPEN;
  this->target_operation_ = TARGET_OPERATION_NONE;
  this->operation_started_time_ = changed_at;
  this->last_recompute_time_ = now;
  this->last_publish_time_ = now;
  this->publish_state(false);
}

void SingleControlCover::motor_power_callback_(float power) {
  if (std::isnan(power))
    return;

  const uint32_t now = millis();
  const uint32_t transition_time = this->estimate_transition_time_(now);

  if (!this->motor_power_initialized_) {
    this->motor_power_initialized_ = true;
    this->motor_running_ = power >= this->motor_running_threshold_;
    if (this->motor_running_)
      this->start_power_detected_operation_(now, power);
  } else if (!this->motor_running_ && power >= this->motor_running_threshold_) {
    ESP_LOGD(TAG, "Motor started at %.1f W", power);
    this->motor_running_ = true;
    this->start_power_detected_operation_(transition_time, power);
  } else if (this->motor_running_ && power <= this->motor_stopped_threshold_) {
    ESP_LOGD(TAG, "Motor stopped at %.1f W", power);
    this->motor_running_ = false;
    if (this->current_operation != COVER_OPERATION_IDLE) {
      this->apply_delayed_transition_(now, transition_time, COVER_OPERATION_IDLE);
      this->finish_operation_(true);
    }
  } else if (this->motor_running_ && this->current_operation == COVER_OPERATION_CLOSING &&
             power <= this->motor_opening_max_power_) {
    // The opener reverses directly from closing to opening without an off-power gap.
    this->change_power_detected_direction_(COVER_OPERATION_OPENING, transition_time, power);
  }

  this->last_motor_sample_time_ = now;
}

}  // namespace sc_cover
}  // namespace esphome
