import esphome.codegen as cg
from esphome.components import binary_sensor
import esphome.config_validation as cv
from esphome.const import (
    DEVICE_CLASS_CONNECTIVITY,
    DEVICE_CLASS_PROBLEM,
    DEVICE_CLASS_RUNNING,
    ENTITY_CATEGORY_DIAGNOSTIC,
)

from . import CONF_MCZ_MAESTRO_BLE_ID, HUB_SCHEMA

SENSORS = {
    "connected": binary_sensor.binary_sensor_schema(
        device_class=DEVICE_CLASS_CONNECTIVITY, entity_category=ENTITY_CATEGORY_DIAGNOSTIC
    ),
    "running": binary_sensor.binary_sensor_schema(device_class=DEVICE_CLASS_RUNNING),
    "alarm": binary_sensor.binary_sensor_schema(device_class=DEVICE_CLASS_PROBLEM),
    "chrono": binary_sensor.binary_sensor_schema(icon="mdi:calendar-clock"),
}

CONFIG_SCHEMA = HUB_SCHEMA.extend({cv.Optional(k): s for k, s in SENSORS.items()})


async def to_code(config):
    hub = await cg.get_variable(config[CONF_MCZ_MAESTRO_BLE_ID])
    for key in SENSORS:
        if key in config:
            sens = await binary_sensor.new_binary_sensor(config[key])
            cg.add(getattr(hub, f"set_{key}_binary_sensor")(sens))
