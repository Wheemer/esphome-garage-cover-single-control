# ESPHome Garage Cover Single Control

Project to control a garage cover or gate with [ESPHome](https://esphome.io/)
and [Home Assistant](https://www.home-assistant.io/).

This project uses:
* Esp board [compatible](https://esphome.io/#devices) with ESPHome
* Relay to activate the cover control
* Two reed switches to detect end positions of the door.

## Cover description

Cover is controlled with a single control using a relay. Each time the control is activated, it performs an action according following
state machine:

* Sequence: open -> stop -> close -> stop -> open
* When cover reach the end (open or close) it counts as a stop action

## Project features

* Position control
* Calculate the number of times the control need to be activated to perform the action requested or reach requested position
* Actuate the door many times as needed to perform requested action. For example if position in memory is wrong or unknow because a external control stops the door at middle.
* Detect and update position when the cover is externally commanded from either endstop.
* Optionally use a motor power sensor to detect external starts and stops at partial positions, including a direct closing-to-opening reversal when each direction has a distinct measured power range.
* Restore the last calculated position after a reboot when neither endstop is active.
* Preserve the calculated position when a stop or toggle command is received.
* Optionally leave a stale moving state after `operation_timeout` without falsely confirming an endstop.
* Configuration options for GPIOs, debounce time, open/close durations. time between control actuation...

## Instructions

* Use this repo as an [external component](https://esphome.io/components/external_components)
* Check the [example](example.yaml) provided in this repo

When configured, `motor_power_sensor` uses separate running and stopped thresholds for hysteresis. Power transitions are timestamped between adjacent samples so brief reporting latency does not accumulate directly into the calculated position.

For openers that reverse when pressed while closing, `closing_stop_delay` sends the required second press after the reversal. Partial closing targets compensate for that brief reverse travel so the door stops near the requested position.
