import esphome.codegen as cg
from esphome import pins
from esphome.components import (
    binary_sensor,
    number,
    sensor,
    switch,
    text_sensor,
    uart,
    web_server_base,
)
from esphome.components.web_server_base import CONF_WEB_SERVER_BASE_ID
import esphome.config_validation as cv
from esphome.const import (
    CONF_CURRENT,
    CONF_FLOW_CONTROL_PIN,
    CONF_ID,
    CONF_NAME,
    CONF_PASSWORD,
    CONF_PORT,
    CONF_POWER,
    CONF_ENERGY,
    CONF_STATE,
    CONF_STEP,
    CONF_UNIT_OF_MEASUREMENT,
    CONF_USERNAME,
    DEVICE_CLASS_BATTERY_CHARGING,
    DEVICE_CLASS_CONNECTIVITY,
    DEVICE_CLASS_CURRENT,
    DEVICE_CLASS_ENERGY,
    DEVICE_CLASS_PLUG,
    DEVICE_CLASS_POWER,
    DEVICE_CLASS_PROBLEM,
    DEVICE_CLASS_VOLTAGE,
    ENTITY_CATEGORY_DIAGNOSTIC,
    STATE_CLASS_MEASUREMENT,
    STATE_CLASS_TOTAL_INCREASING,
    UNIT_AMPERE,
    UNIT_KILOWATT,
    UNIT_KILOWATT_HOURS,
    UNIT_VOLT,
)
from esphome.cpp_helpers import gpio_pin_expression

CONF_MAX_ALLOWABLE_CURRENT = "max_allowable_current"
CONF_TOTAL_KWH = "total_kwh_delivered"
CONF_SERIAL = "serial"
CONF_PHASE_1_VOLTAGE = "phase_1_voltage"
CONF_PHASE_2_VOLTAGE = "phase_2_voltage"
CONF_PHASE_3_VOLTAGE = "phase_3_voltage"
CONF_PHASE_1_CURRENT = "phase_1_current"
CONF_PHASE_2_CURRENT = "phase_2_current"
CONF_PHASE_3_CURRENT = "phase_3_current"
CONF_FIRMWARE_VERSION = "firmware_version"
CONF_ACTUAL_CURRENT = "actual_current"
CONF_OFFERED_CURRENT = "offered_current"
CONF_VIN = "connected_vin"
CONF_PASSIVE_MODE = "passive_mode"

CONF_MIN_CURRENT = "min_current"
CONF_MAX_CURRENT = "max_current"
CONF_SET_CURRENT = "set_current"
CONF_FAILSAFE_CURRENT = "failsafe_current"
CONF_SETPOINT_TIMEOUT = "setpoint_timeout"
CONF_MIN_ON_TIME = "min_on_time"
CONF_MIN_OFF_TIME = "min_off_time"
CONF_REASSERT_CURRENT = "reassert_current"
CONF_USE_START_STOP = "use_start_stop_commands"
CONF_QUERY_VIN = "query_vin"
CONF_PHASES = "phases"
CONF_NOMINAL_VOLTAGE = "nominal_voltage"
CONF_TWCID = "twc_id"
CONF_ALLOW_CHARGING = "allow_charging"
CONF_VEHICLE_CONNECTED = "vehicle_connected"
CONF_CHARGING_ACTIVE = "charging_active"
CONF_VEHICLE_CONNECTED_NUMERIC = "vehicle_connected_numeric"
CONF_CHARGING_ACTIVE_NUMERIC = "charging_active_numeric"
CONF_LINK = "link"
CONF_FAILSAFE = "failsafe"

CONF_LOXONE = "loxone"
CONF_MINISERVER = "miniserver"
CONF_API_INPUT = "api_input"
CONF_BLOCK = "block"
CONF_VC_INPUT = "vc_input"
CONF_MIN_INTERVAL = "min_interval"
CONF_KEEPALIVE = "keepalive"
CONF_FEED_WATCHDOG = "feed_watchdog"

ICON_CURRENT_AC = "mdi:current-ac"
ICON_CAR = "mdi:car"
ICON_NUMERIC = "mdi:numeric"
ICON_LIGHTNING_BOLT = "mdi:lightning-bolt"
ICON_EV_STATION = "mdi:ev-station"
ICON_BATTERY_CHARGING = "mdi:battery-charging"

DEPENDENCIES = ["uart"]


def AUTO_LOAD(config):
    load = ["binary_sensor", "number", "sensor", "switch", "text_sensor"]
    if config and CONF_LOXONE in config:
        load += ["web_server_base", "network"]
    return load


TEXT_TYPES = [
    CONF_SERIAL,
    CONF_FIRMWARE_VERSION,
    CONF_VIN,
]

TYPES = [
    CONF_MAX_ALLOWABLE_CURRENT,
    CONF_CURRENT,
    CONF_STATE,
    CONF_TOTAL_KWH,
    CONF_PHASE_1_VOLTAGE,
    CONF_PHASE_2_VOLTAGE,
    CONF_PHASE_3_VOLTAGE,
    CONF_PHASE_1_CURRENT,
    CONF_PHASE_2_CURRENT,
    CONF_PHASE_3_CURRENT,
    CONF_ACTUAL_CURRENT,
    CONF_OFFERED_CURRENT,
    CONF_POWER,
    CONF_ENERGY,
    CONF_VEHICLE_CONNECTED_NUMERIC,
    CONF_CHARGING_ACTIVE_NUMERIC,
]

BINARY_TYPES = [
    CONF_VEHICLE_CONNECTED,
    CONF_CHARGING_ACTIVE,
    CONF_LINK,
    CONF_FAILSAFE,
]

twc_gateway_ns = cg.esphome_ns.namespace("twc_gateway")
TWCGateway = twc_gateway_ns.class_(
    "TWCGateway", number.Number, cg.Component, uart.UARTDevice
)
AllowChargingSwitch = twc_gateway_ns.class_("AllowChargingSwitch", switch.Switch)
LoxoneBridge = twc_gateway_ns.class_("LoxoneBridge", cg.Component)


def _boolean_or_01(value):
    # passive_mode used to be documented as 0/1; keep accepting that.
    if isinstance(value, int) and not isinstance(value, bool) and value in (0, 1):
        return bool(value)
    return cv.boolean(value)


def validate_currents(config):
    min_current = config[CONF_MIN_CURRENT]
    max_current = config[CONF_MAX_CURRENT]
    if 0 < min_current < 6:
        raise cv.Invalid(
            f"{CONF_MIN_CURRENT} must be 0 (allow offering nothing) or at least 6 - "
            "the protocol has no valid current between 0 and 6 A"
        )
    if max_current <= min_current:
        raise cv.Invalid(f"{CONF_MAX_CURRENT} must be greater than {CONF_MIN_CURRENT}")
    if CONF_SET_CURRENT in config:
        if not min_current <= config[CONF_SET_CURRENT] <= max_current:
            raise cv.Invalid(
                f"{CONF_SET_CURRENT} must be between {CONF_MIN_CURRENT} and {CONF_MAX_CURRENT}"
            )
    failsafe = config[CONF_FAILSAFE_CURRENT]
    if failsafe > max_current or 0 < failsafe < 6:
        raise cv.Invalid(f"{CONF_FAILSAFE_CURRENT} must be 0 or between 6 and {CONF_MAX_CURRENT}")
    return config


def _amps_sensor():
    return sensor.sensor_schema(
        unit_of_measurement=UNIT_AMPERE,
        icon=ICON_CURRENT_AC,
        accuracy_decimals=1,
        device_class=DEVICE_CLASS_CURRENT,
        state_class=STATE_CLASS_MEASUREMENT,
    )


def _volts_sensor():
    return sensor.sensor_schema(
        unit_of_measurement=UNIT_VOLT,
        icon=ICON_LIGHTNING_BOLT,
        accuracy_decimals=0,
        device_class=DEVICE_CLASS_VOLTAGE,
        state_class=STATE_CLASS_MEASUREMENT,
    )


LOXONE_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(LoxoneBridge),
        cv.GenerateID(CONF_WEB_SERVER_BASE_ID): cv.use_id(web_server_base.WebServerBase),
        # Push to the Miniserver is optional; without these only the
        # /loxone and /loxone/set endpoints are served.
        cv.Optional(CONF_MINISERVER): cv.ipv4address,
        cv.Optional(CONF_PORT, default=80): cv.port,
        cv.Optional(CONF_USERNAME, default=""): cv.string,
        cv.Optional(CONF_PASSWORD, default=""): cv.string,
        cv.Optional(CONF_API_INPUT): cv.string_strict,
        cv.Optional(CONF_BLOCK, default="Wb"): cv.string_strict,
        cv.Optional(CONF_VC_INPUT): cv.string_strict,
        cv.Optional(CONF_MIN_INTERVAL, default="5s"): cv.positive_time_period_milliseconds,
        cv.Optional(CONF_KEEPALIVE, default="30s"): cv.positive_time_period_milliseconds,
        cv.Optional(CONF_FEED_WATCHDOG, default=True): cv.boolean,
    }
)


def validate_loxone(config):
    lox = config.get(CONF_LOXONE)
    if lox is None:
        return config
    if (CONF_MINISERVER in lox) != (CONF_API_INPUT in lox):
        raise cv.Invalid(
            f"{CONF_LOXONE}: set both {CONF_MINISERVER} and {CONF_API_INPUT} to push to the "
            "Miniserver, or neither to only serve /loxone"
        )
    return config


CONFIG_SCHEMA = cv.All(
    number.number_schema(TWCGateway)
    .extend(
        {
            cv.Optional(CONF_NAME, default="Set Current"): cv.string,
            cv.Optional(CONF_MIN_CURRENT, default=6): cv.int_range(min=0, max=32),
            cv.Optional(CONF_MAX_CURRENT, default=32): cv.int_range(min=6, max=80),
            cv.Optional(CONF_TWCID, default=0xABCD): cv.hex_int_range(0, 65535),
            cv.Optional(CONF_SET_CURRENT): cv.int_range(min=0, max=80),
            cv.Optional(CONF_STEP, default=1): cv.positive_int,
            cv.Optional(CONF_UNIT_OF_MEASUREMENT, default=UNIT_AMPERE): cv.one_of(UNIT_AMPERE),
            cv.Optional(CONF_PASSIVE_MODE, default=False): _boolean_or_01,
            cv.Optional(CONF_FLOW_CONTROL_PIN): pins.gpio_output_pin_schema,
            # Safety: what to offer when nothing has commanded us for
            # setpoint_timeout (0 s = watchdog off).
            cv.Optional(CONF_FAILSAFE_CURRENT, default=0): cv.int_range(min=0, max=80),
            cv.Optional(CONF_SETPOINT_TIMEOUT, default="0s"): cv.positive_time_period_milliseconds,
            # Relay protection, as in TWCManager.
            cv.Optional(CONF_MIN_ON_TIME, default="60s"): cv.positive_time_period_milliseconds,
            cv.Optional(CONF_MIN_OFF_TIME, default="30s"): cv.positive_time_period_milliseconds,
            # Legacy: send the limit in every heartbeat instead of until the
            # TWC confirms it.
            cv.Optional(CONF_REASSERT_CURRENT, default=False): cv.boolean,
            # Experimental FCB1/FCB2 start/stop frames on top of the 0 A limit.
            cv.Optional(CONF_USE_START_STOP, default=False): cv.boolean,
            cv.Optional(CONF_QUERY_VIN, default=True): cv.boolean,
            cv.Optional(CONF_PHASES, default=3): cv.int_range(min=1, max=3),
            cv.Optional(CONF_NOMINAL_VOLTAGE, default=230.0): cv.float_range(min=100, max=260),
            cv.Optional(CONF_CURRENT): _amps_sensor(),
            cv.Optional(CONF_MAX_ALLOWABLE_CURRENT): _amps_sensor(),
            cv.Optional(CONF_TOTAL_KWH): sensor.sensor_schema(
                unit_of_measurement=UNIT_KILOWATT_HOURS,
                icon=ICON_LIGHTNING_BOLT,
                accuracy_decimals=0,
                device_class=DEVICE_CLASS_ENERGY,
                state_class=STATE_CLASS_TOTAL_INCREASING,
            ),
            cv.Optional(CONF_ENERGY): sensor.sensor_schema(
                unit_of_measurement=UNIT_KILOWATT_HOURS,
                icon=ICON_LIGHTNING_BOLT,
                accuracy_decimals=3,
                device_class=DEVICE_CLASS_ENERGY,
                state_class=STATE_CLASS_TOTAL_INCREASING,
            ),
            cv.Optional(CONF_POWER): sensor.sensor_schema(
                unit_of_measurement=UNIT_KILOWATT,
                icon=ICON_LIGHTNING_BOLT,
                accuracy_decimals=2,
                device_class=DEVICE_CLASS_POWER,
                state_class=STATE_CLASS_MEASUREMENT,
            ),
            cv.Optional(CONF_SERIAL): text_sensor.text_sensor_schema(
                icon=ICON_EV_STATION,
                entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
            ),
            cv.Optional(CONF_PHASE_1_VOLTAGE): _volts_sensor(),
            cv.Optional(CONF_PHASE_2_VOLTAGE): _volts_sensor(),
            cv.Optional(CONF_PHASE_3_VOLTAGE): _volts_sensor(),
            cv.Optional(CONF_PHASE_1_CURRENT): _amps_sensor(),
            cv.Optional(CONF_PHASE_2_CURRENT): _amps_sensor(),
            cv.Optional(CONF_PHASE_3_CURRENT): _amps_sensor(),
            cv.Optional(CONF_FIRMWARE_VERSION): text_sensor.text_sensor_schema(
                icon=ICON_NUMERIC,
                entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
            ),
            cv.Optional(CONF_VIN): text_sensor.text_sensor_schema(
                icon=ICON_CAR,
            ),
            cv.Optional(CONF_ACTUAL_CURRENT): _amps_sensor(),
            cv.Optional(CONF_OFFERED_CURRENT): _amps_sensor(),
            cv.Optional(CONF_STATE): sensor.sensor_schema(
                icon=ICON_EV_STATION,
                accuracy_decimals=0,
            ),
            cv.Optional(CONF_ALLOW_CHARGING): switch.switch_schema(
                AllowChargingSwitch,
                icon=ICON_EV_STATION,
                default_restore_mode="RESTORE_DEFAULT_ON",
            ),
            cv.Optional(CONF_VEHICLE_CONNECTED): binary_sensor.binary_sensor_schema(
                icon=ICON_CAR,
                device_class=DEVICE_CLASS_PLUG,
            ),
            cv.Optional(CONF_CHARGING_ACTIVE): binary_sensor.binary_sensor_schema(
                icon=ICON_BATTERY_CHARGING,
                device_class=DEVICE_CLASS_BATTERY_CHARGING,
            ),
            cv.Optional(CONF_LINK): binary_sensor.binary_sensor_schema(
                device_class=DEVICE_CLASS_CONNECTIVITY,
                entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
            ),
            cv.Optional(CONF_FAILSAFE): binary_sensor.binary_sensor_schema(
                device_class=DEVICE_CLASS_PROBLEM,
                entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
            ),
            cv.Optional(CONF_VEHICLE_CONNECTED_NUMERIC): sensor.sensor_schema(
                icon=ICON_CAR,
                accuracy_decimals=0,
            ),
            cv.Optional(CONF_CHARGING_ACTIVE_NUMERIC): sensor.sensor_schema(
                icon=ICON_BATTERY_CHARGING,
                accuracy_decimals=0,
            ),
            cv.Optional(CONF_LOXONE): LOXONE_SCHEMA,
        }
    )
    .extend(uart.UART_DEVICE_SCHEMA)
    .extend(cv.COMPONENT_SCHEMA),
    validate_currents,
    validate_loxone,
)


async def to_code(config):
    num_var = await number.new_number(
        config,
        min_value=config[CONF_MIN_CURRENT],
        max_value=config[CONF_MAX_CURRENT],
        step=config[CONF_STEP],
    )

    await cg.register_component(num_var, config)
    await uart.register_uart_device(num_var, config)

    cg.add(num_var.set_min_current(max(config[CONF_MIN_CURRENT], 6)))
    cg.add(num_var.set_max_current(config[CONF_MAX_CURRENT]))
    cg.add(num_var.set_twcid(config[CONF_TWCID]))
    cg.add(num_var.set_passive_mode(config[CONF_PASSIVE_MODE]))
    cg.add(num_var.set_failsafe_current(config[CONF_FAILSAFE_CURRENT]))
    cg.add(num_var.set_setpoint_timeout(config[CONF_SETPOINT_TIMEOUT]))
    cg.add(num_var.set_min_on_time(config[CONF_MIN_ON_TIME]))
    cg.add(num_var.set_min_off_time(config[CONF_MIN_OFF_TIME]))
    cg.add(num_var.set_reassert_current(config[CONF_REASSERT_CURRENT]))
    cg.add(num_var.set_use_start_stop_commands(config[CONF_USE_START_STOP]))
    cg.add(num_var.set_query_vin(config[CONF_QUERY_VIN]))
    cg.add(num_var.set_phases(config[CONF_PHASES]))
    cg.add(num_var.set_nominal_voltage(config[CONF_NOMINAL_VOLTAGE]))
    # Value offered after boot until something writes the number entity.
    cg.add(num_var.set_initial_current(config.get(CONF_SET_CURRENT, config[CONF_MIN_CURRENT])))

    for key in TYPES:
        if sensor_config := config.get(key):
            sens = await sensor.new_sensor(sensor_config)
            cg.add(getattr(num_var, f"set_{key}_sensor")(sens))

    for key in TEXT_TYPES:
        if sensor_config := config.get(key):
            sens = await text_sensor.new_text_sensor(sensor_config)
            cg.add(getattr(num_var, f"set_{key}_text_sensor")(sens))

    for key in BINARY_TYPES:
        if sensor_config := config.get(key):
            sens = await binary_sensor.new_binary_sensor(sensor_config)
            cg.add(getattr(num_var, f"set_{key}_binary_sensor")(sens))

    if allow_charging_config := config.get(CONF_ALLOW_CHARGING):
        allow_charging_switch = await switch.new_switch(allow_charging_config)
        cg.add(allow_charging_switch.set_parent(num_var))
        cg.add(num_var.set_allow_charging_switch(allow_charging_switch))

    if CONF_FLOW_CONTROL_PIN in config:
        pin = await gpio_pin_expression(config[CONF_FLOW_CONTROL_PIN])
        cg.add(num_var.set_flow_control_pin(pin))

    if lox := config.get(CONF_LOXONE):
        cg.add_define("USE_TWC_LOXONE")
        base = await cg.get_variable(lox[CONF_WEB_SERVER_BASE_ID])
        bridge = cg.new_Pvariable(lox[CONF_ID], num_var, base)
        await cg.register_component(bridge, lox)
        if CONF_MINISERVER in lox:
            cg.add(bridge.set_miniserver(str(lox[CONF_MINISERVER]), lox[CONF_PORT]))
            cg.add(bridge.set_api_input(lox[CONF_API_INPUT]))
        cg.add(bridge.set_credentials(lox[CONF_USERNAME], lox[CONF_PASSWORD]))
        cg.add(bridge.set_block(lox[CONF_BLOCK]))
        if CONF_VC_INPUT in lox:
            cg.add(bridge.set_vc_input(lox[CONF_VC_INPUT]))
        cg.add(bridge.set_min_interval(lox[CONF_MIN_INTERVAL]))
        cg.add(bridge.set_keepalive(lox[CONF_KEEPALIVE]))
        cg.add(bridge.set_feed_watchdog(lox[CONF_FEED_WATCHDOG]))
