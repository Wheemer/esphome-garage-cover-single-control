import esphome.codegen as cg
from esphome.components import binary_sensor, button, cover, sensor
import esphome.config_validation as cv
from esphome.const import (
    CONF_CLOSE_DURATION,
    CONF_CLOSE_ENDSTOP,
    CONF_ID,
    CONF_OPEN_DURATION,
    CONF_OPEN_ENDSTOP,
)

CONF_DOOR_ACTIVATE_BUTTON = "door_activate_button"
CONF_BUTTON_PRESS_INTERVAL = "button_press_interval"
CONF_CLOSING_STOP_DELAY = "closing_stop_delay"
CONF_SETUP_DELAY = "setup_delay"
CONF_OPERATION_TIMEOUT = "operation_timeout"
CONF_MOTOR_POWER_SENSOR = "motor_power_sensor"
CONF_MOTOR_RUNNING_THRESHOLD = "motor_running_threshold"
CONF_MOTOR_STOPPED_THRESHOLD = "motor_stopped_threshold"
CONF_MOTOR_OPENING_MAX_POWER = "motor_opening_max_power"
CONF_MOTOR_CLOSING_MIN_POWER = "motor_closing_min_power"

sc_cover_ns = cg.esphome_ns.namespace("sc_cover")
SingleControlCover = sc_cover_ns.class_("SingleControlCover", cover.Cover, cg.Component)


def validate_motor_thresholds(config):
    if CONF_MOTOR_POWER_SENSOR in config:
        stopped = config[CONF_MOTOR_STOPPED_THRESHOLD]
        running = config[CONF_MOTOR_RUNNING_THRESHOLD]
        opening_max = config[CONF_MOTOR_OPENING_MAX_POWER]
        closing_min = config[CONF_MOTOR_CLOSING_MIN_POWER]
        if stopped >= running:
            raise cv.Invalid("motor_stopped_threshold must be lower than motor_running_threshold")
        if running >= opening_max:
            raise cv.Invalid("motor_running_threshold must be lower than motor_opening_max_power")
        if opening_max >= closing_min:
            raise cv.Invalid("motor_opening_max_power must be lower than motor_closing_min_power")
    return config


CONFIG_SCHEMA = cv.All(
    (
    cover.cover_schema(SingleControlCover)
    .extend(
        {
            cv.Required(CONF_DOOR_ACTIVATE_BUTTON): cv.use_id(button.Button),
            cv.Required(
                CONF_BUTTON_PRESS_INTERVAL
            ): cv.positive_time_period_milliseconds,
            cv.Optional(CONF_CLOSING_STOP_DELAY, default="1100ms"): cv.positive_time_period_milliseconds,
            cv.Required(CONF_OPEN_ENDSTOP): cv.use_id(binary_sensor.BinarySensor),
            cv.Required(CONF_OPEN_DURATION): cv.positive_time_period_milliseconds,
            cv.Required(CONF_CLOSE_ENDSTOP): cv.use_id(binary_sensor.BinarySensor),
            cv.Required(CONF_CLOSE_DURATION): cv.positive_time_period_milliseconds,
            cv.Optional(CONF_SETUP_DELAY): cv.positive_time_period_milliseconds,
            cv.Optional(CONF_OPERATION_TIMEOUT): cv.positive_time_period_milliseconds,
            cv.Optional(CONF_MOTOR_POWER_SENSOR): cv.use_id(sensor.Sensor),
            cv.Optional(CONF_MOTOR_RUNNING_THRESHOLD, default=100.0): cv.float_range(min=0),
            cv.Optional(CONF_MOTOR_STOPPED_THRESHOLD, default=50.0): cv.float_range(min=0),
            cv.Optional(CONF_MOTOR_OPENING_MAX_POWER, default=380.0): cv.float_range(min=0),
            cv.Optional(CONF_MOTOR_CLOSING_MIN_POWER, default=390.0): cv.float_range(min=0),
        }
    )
    .extend(cv.COMPONENT_SCHEMA)
    ),
    validate_motor_thresholds,
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await cover.register_cover(var, config)

    setup_delay = config.get(CONF_SETUP_DELAY)
    if setup_delay:
        cg.add(var.set_setup_delay(setup_delay))

    operation_timeout = config.get(CONF_OPERATION_TIMEOUT)
    if operation_timeout:
        cg.add(var.set_operation_timeout(operation_timeout))

    motor_power_sensor = config.get(CONF_MOTOR_POWER_SENSOR)
    if motor_power_sensor:
        power = await cg.get_variable(motor_power_sensor)
        cg.add(var.set_motor_power_sensor(power))
        cg.add(var.set_motor_running_threshold(config[CONF_MOTOR_RUNNING_THRESHOLD]))
        cg.add(var.set_motor_stopped_threshold(config[CONF_MOTOR_STOPPED_THRESHOLD]))
        cg.add(var.set_motor_opening_max_power(config[CONF_MOTOR_OPENING_MAX_POWER]))
        cg.add(var.set_motor_closing_min_power(config[CONF_MOTOR_CLOSING_MIN_POWER]))

    bin = await cg.get_variable(config[CONF_DOOR_ACTIVATE_BUTTON])
    cg.add(var.set_door_activate_button(bin))
    cg.add(var.set_button_press_interval(config[CONF_BUTTON_PRESS_INTERVAL]))
    cg.add(var.set_closing_stop_delay(config[CONF_CLOSING_STOP_DELAY]))

    bin = await cg.get_variable(config[CONF_OPEN_ENDSTOP])
    cg.add(var.set_open_endstop(bin))
    cg.add(var.set_open_duration(config[CONF_OPEN_DURATION]))

    bin = await cg.get_variable(config[CONF_CLOSE_ENDSTOP])
    cg.add(var.set_close_endstop(bin))
    cg.add(var.set_close_duration(config[CONF_CLOSE_DURATION]))
