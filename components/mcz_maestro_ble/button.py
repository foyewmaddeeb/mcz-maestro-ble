import esphome.codegen as cg
from esphome.components import button
import esphome.config_validation as cv
from esphome.const import ENTITY_CATEGORY_CONFIG

from . import CONF_MCZ_MAESTRO_BLE_ID, HUB_SCHEMA, mcz_ns

MczButton = mcz_ns.class_("MczButton", button.Button)
ButtonType = mcz_ns.enum("ButtonType", is_class=True)

BUTTONS = {
    "alarm_reset": ("ALARM_RESET", "mdi:alert-remove-outline"),
    "sync_clock": ("SYNC_CLOCK", "mdi:clock-check-outline"),
}

CONFIG_SCHEMA = HUB_SCHEMA.extend(
    {
        cv.Optional(k): button.button_schema(
            MczButton, icon=icon, entity_category=ENTITY_CATEGORY_CONFIG
        )
        for k, (_, icon) in BUTTONS.items()
    }
)


async def to_code(config):
    hub = await cg.get_variable(config[CONF_MCZ_MAESTRO_BLE_ID])
    for key, (member, _) in BUTTONS.items():
        if key in config:
            var = await button.new_button(config[key])
            cg.add(var.set_parent(hub))
            cg.add(var.set_type(getattr(ButtonType, member)))
