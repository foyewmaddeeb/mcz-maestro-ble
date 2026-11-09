import esphome.codegen as cg
from esphome.components import text_sensor
import esphome.config_validation as cv
from esphome.const import ENTITY_CATEGORY_DIAGNOSTIC

from . import CONF_MCZ_MAESTRO_BLE_ID, HUB_SCHEMA

SENSORS = {
    "state": text_sensor.text_sensor_schema(icon="mdi:fireplace"),
    "mode": text_sensor.text_sensor_schema(icon="mdi:thermostat-auto"),
    "serial_number": text_sensor.text_sensor_schema(
        icon="mdi:identifier", entity_category=ENTITY_CATEGORY_DIAGNOSTIC
    ),
    "model": text_sensor.text_sensor_schema(
        icon="mdi:information-outline", entity_category=ENTITY_CATEGORY_DIAGNOSTIC
    ),
    "alarm_history": text_sensor.text_sensor_schema(
        icon="mdi:history", entity_category=ENTITY_CATEGORY_DIAGNOSTIC
    ),
}

CONFIG_SCHEMA = HUB_SCHEMA.extend({cv.Optional(k): s for k, s in SENSORS.items()})


async def to_code(config):
    hub = await cg.get_variable(config[CONF_MCZ_MAESTRO_BLE_ID])
    for key in SENSORS:
        if key in config:
            sens = await text_sensor.new_text_sensor(config[key])
            cg.add(getattr(hub, f"set_{key}_text_sensor")(sens))
