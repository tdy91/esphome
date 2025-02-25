from esphome import automation
import esphome.codegen as cg
from esphome.components import number, output, sensor
import esphome.config_validation as cv
from esphome.const import (
    CONF_ENTITY_CATEGORY,
    CONF_ID,
    CONF_INITIAL_VALUE,
    CONF_MAX_VALUE,
    CONF_MIN_VALUE,
    CONF_NAME,
    CONF_RESTORE_VALUE,
    CONF_SET_ACTION,
    CONF_STEP,
    ENTITY_CATEGORY_CONFIG,
)

CODEOWNERS = ["@tdy91"]

DEPENDENCIES = ["output"]
CONF_SENSOR_ROOM_TEMPERATURE = "sensor_room_temperature"
CONF_SENSOR_OUTDOOR_TEMPERATURE = "sensor_outdoor_temperature"
CONF_KP = "kp"
CONF_TI = "ti"
CONF_TD = "td"
CONF_N = "n"
CONF_KAW = "kaw"
CONF_SAMPLE_TIME_S = "sample_time_s"
CONF_KP_EXT = "kp_ext"
CONF_TI_EXT = "ti_ext"
CONF_TD_EXT = "td_ext"
CONF_N_EXT = "n_ext"
CONF_TARGET_TEMP_NUMBER = "target_temp_number"
CONF_TARGET_TEMPERATURE_NUMBER_ID = "target_temperature_number_id"
CONF_THERMAL_LOAD_CORRECTION_NUMBER_ID = "thermal_load_correction_number_id"
CONF_FORECAST_WEATHER_CORRECTION_NUMBER_ID = "forecast_weather_correction_number_id"
CONF_HEATER_OFF_OUTDOOR_TEMPERATURE = "heater_off_outdoor_temperature"
CONF_TRV_OUTPUT_ID = "trv_output_id"


trv_pid_ns = cg.esphome_ns.namespace("trv_pid")
trv_pid = trv_pid_ns.class_("trv_pid", cg.Component)
TargetTemperatureNumber = trv_pid_ns.class_(
    "TargetTemperatureNumber", number.Number, cg.Component
)


def validate_min_max(config):
    if config[CONF_MAX_VALUE] <= config[CONF_MIN_VALUE]:
        raise cv.Invalid(f"{CONF_MAX_VALUE} must be greater than {CONF_MIN_VALUE}")

    if (config[CONF_INITIAL_VALUE] > config[CONF_MAX_VALUE]) or (
        config[CONF_INITIAL_VALUE] < config[CONF_MIN_VALUE]
    ):
        raise cv.Invalid(
            f"{CONF_INITIAL_VALUE} must be a value between {CONF_MAX_VALUE} and {CONF_MIN_VALUE}"
        )
    return config


CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(trv_pid),
        cv.Required(CONF_SENSOR_ROOM_TEMPERATURE): cv.use_id(sensor.Sensor),
        cv.Required(CONF_SENSOR_OUTDOOR_TEMPERATURE): cv.use_id(sensor.Sensor),
        cv.Optional(CONF_TARGET_TEMP_NUMBER): cv.maybe_simple_value(
            number.NUMBER_SCHEMA.extend(
                {
                    cv.GenerateID(): cv.declare_id(TargetTemperatureNumber),
                    cv.Optional(
                        CONF_ENTITY_CATEGORY, default=ENTITY_CATEGORY_CONFIG
                    ): cv.entity_category,
                    cv.Optional(CONF_INITIAL_VALUE, default=1): cv.positive_float,
                    cv.Optional(CONF_MAX_VALUE, default=10): cv.positive_float,
                    cv.Optional(CONF_MIN_VALUE, default=0): cv.positive_float,
                    cv.Optional(CONF_RESTORE_VALUE, default=True): cv.boolean,
                    cv.Optional(CONF_STEP, default=0.1): cv.positive_float,
                    cv.Optional(CONF_SET_ACTION): automation.validate_automation(
                        single=True
                    ),
                }
            ).extend(cv.COMPONENT_SCHEMA),
            validate_min_max,
            key=CONF_NAME,
        ),
        cv.Required(CONF_TARGET_TEMPERATURE_NUMBER_ID): cv.use_id(number.Number),
        cv.Required(CONF_THERMAL_LOAD_CORRECTION_NUMBER_ID): cv.use_id(number.Number),
        cv.Required(CONF_FORECAST_WEATHER_CORRECTION_NUMBER_ID): cv.use_id(
            number.Number
        ),
        cv.Required(CONF_TRV_OUTPUT_ID): cv.use_id(output.FloatOutput),
        cv.Optional(CONF_HEATER_OFF_OUTDOOR_TEMPERATURE, default=11.0): cv.float_,
        cv.Optional(CONF_KP, default=0.39): cv.float_,
        cv.Optional(CONF_TI, default=1800): cv.float_,
        cv.Optional(CONF_TD, default=300): cv.float_,
        cv.Optional(CONF_N, default=10.0): cv.float_,
        cv.Optional(CONF_KAW, default=1.0): cv.float_,
        cv.Optional(CONF_SAMPLE_TIME_S, default=15): cv.uint32_t,
        cv.Optional(CONF_KP_EXT, default=0.06): cv.float_,
        cv.Optional(CONF_TI_EXT, default=1800): cv.float_,
        cv.Optional(CONF_TD_EXT, default=300): cv.float_,
        cv.Optional(CONF_N_EXT, default=10.0): cv.float_,
    }
).extend(cv.COMPONENT_SCHEMA)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID], config[CONF_SAMPLE_TIME_S])
    await cg.register_component(var, config)
    room_sensor = await cg.get_variable(config[CONF_SENSOR_ROOM_TEMPERATURE])
    outdoor_sensor = await cg.get_variable(config[CONF_SENSOR_OUTDOOR_TEMPERATURE])
    cg.add(var.set_room_temperature_sensor(room_sensor))
    cg.add(var.set_outdoor_temperature_sensor(outdoor_sensor))
    trv_output = await cg.get_variable(config[CONF_TRV_OUTPUT_ID])
    cg.add(var.set_trv_output(trv_output))
    if CONF_TARGET_TEMP_NUMBER in config:
        target_temp_number_var = await number.new_number(
            config[CONF_TARGET_TEMP_NUMBER],
            min_value=config[CONF_TARGET_TEMP_NUMBER][CONF_MIN_VALUE],
            max_value=config[CONF_TARGET_TEMP_NUMBER][CONF_MAX_VALUE],
            step=config[CONF_TARGET_TEMP_NUMBER][CONF_STEP],
        )
        await cg.register_component(
            target_temp_number_var, config[CONF_TARGET_TEMP_NUMBER]
        )
        cg.add(
            target_temp_number_var.set_initial_value(
                config[CONF_TARGET_TEMP_NUMBER][CONF_INITIAL_VALUE]
            )
        )
        cg.add(
            target_temp_number_var.set_restore_value(
                config[CONF_TARGET_TEMP_NUMBER][CONF_RESTORE_VALUE]
            )
        )
        if CONF_SET_ACTION in config[CONF_TARGET_TEMP_NUMBER]:
            await automation.build_automation(
                target_temp_number_var.get_set_trigger(),
                [(float, "x")],
                config[CONF_TARGET_TEMP_NUMBER][CONF_SET_ACTION],
            )
        cg.add(var.set_target_temp_number(target_temp_number_var))

    target_temp_number = await cg.get_variable(
        config[CONF_TARGET_TEMPERATURE_NUMBER_ID]
    )
    cg.add(var.set_target_temperature_number(target_temp_number))
    thermal_load_correction_number = await cg.get_variable(
        config[CONF_THERMAL_LOAD_CORRECTION_NUMBER_ID]
    )
    cg.add(var.set_thermal_load_correction_number(thermal_load_correction_number))
    forecast_wheather_correction_number = await cg.get_variable(
        config[CONF_FORECAST_WEATHER_CORRECTION_NUMBER_ID]
    )
    cg.add(
        var.set_forecast_wheather_correction_number(forecast_wheather_correction_number)
    )
    cg.add(
        var.set_heater_off_outdoor_temperature(
            config[CONF_HEATER_OFF_OUTDOOR_TEMPERATURE]
        )
    )
    cg.add(var.set_kp(config[CONF_KP]))
    cg.add(var.set_ti(config[CONF_TI]))
    cg.add(var.set_td(config[CONF_TD]))
    cg.add(var.set_n(config[CONF_N]))
    cg.add(var.set_kaw(config[CONF_KAW]))
    cg.add(var.set_kp_ext(config[CONF_KP_EXT]))
    cg.add(var.set_ti_ext(config[CONF_TI_EXT]))
    cg.add(var.set_td_ext(config[CONF_TD_EXT]))
    cg.add(var.set_n_ext(config[CONF_N_EXT]))
