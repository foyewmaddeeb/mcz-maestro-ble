import esphome.codegen as cg
from esphome.components import sensor
import esphome.config_validation as cv
from esphome.const import (
    DEVICE_CLASS_DURATION,
    DEVICE_CLASS_TEMPERATURE,
    ENTITY_CATEGORY_DIAGNOSTIC,
    STATE_CLASS_MEASUREMENT,
    STATE_CLASS_TOTAL_INCREASING,
    UNIT_CELSIUS,
    UNIT_HOUR,
    UNIT_REVOLUTIONS_PER_MINUTE,
)

from . import CONF_MCZ_MAESTRO_BLE_ID, HUB_SCHEMA


def _temp(diag=False):
    kw = {}
    if diag:
        kw["entity_category"] = ENTITY_CATEGORY_DIAGNOSTIC
    return sensor.sensor_schema(
        unit_of_measurement=UNIT_CELSIUS,
        accuracy_decimals=1,
        device_class=DEVICE_CLASS_TEMPERATURE,
        state_class=STATE_CLASS_MEASUREMENT,
        **kw,
    )


def _plain(icon, unit=None, diag=False, state_class=STATE_CLASS_MEASUREMENT):
    kw = {"icon": icon, "accuracy_decimals": 0, "state_class": state_class}
    if unit:
        kw["unit_of_measurement"] = unit
    if diag:
        kw["entity_category"] = ENTITY_CATEGORY_DIAGNOSTIC
    return sensor.sensor_schema(**kw)


SENSORS = {
    "room_temperature": _temp(),
    "fumes_temperature": _temp(),
    "board_temperature": _temp(diag=True),
    "boiler_temperature": _temp(),
    "puffer_temperature": _temp(),
    "setpoint": _temp(),
    "power_level": _plain("mdi:fire"),
    "fan_level": _plain("mdi:fan"),
    "combustion_fan_speed": _plain("mdi:fan", UNIT_REVOLUTIONS_PER_MINUTE, diag=True),
    "exhaust_fan_speed": _plain("mdi:fan", UNIT_REVOLUTIONS_PER_MINUTE, diag=True),
    "ignitions": _plain("mdi:counter", diag=True, state_class=STATE_CLASS_TOTAL_INCREASING),
    "working_time": sensor.sensor_schema(
        unit_of_measurement=UNIT_HOUR,
        accuracy_decimals=1,
        device_class=DEVICE_CLASS_DURATION,
        state_class=STATE_CLASS_TOTAL_INCREASING,
        entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
    ),
    "alarm_code": _plain("mdi:alert-circle-outline", diag=True),
    "active": _plain("mdi:information-outline", diag=True),
}

CONFIG_SCHEMA = HUB_SCHEMA.extend({cv.Optional(k): s for k, s in SENSORS.items()})


async def to_code(config):
    hub = await cg.get_variable(config[CONF_MCZ_MAESTRO_BLE_ID])
    for key in SENSORS:
        if key in config:
            sens = await sensor.new_sensor(config[key])
            cg.add(getattr(hub, f"set_{key}_sensor")(sens))
