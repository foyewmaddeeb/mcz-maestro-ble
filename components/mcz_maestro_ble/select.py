import esphome.codegen as cg
from esphome.components import select
import esphome.config_validation as cv

from . import CONF_MCZ_MAESTRO_BLE_ID, HUB_SCHEMA, mcz_ns

MczSelect = mcz_ns.class_("MczSelect", select.Select)
SelectType = mcz_ns.enum("SelectType", is_class=True)

# key -> (enum member, options, setter on the hub, icon). Option order = register value order.
SELECTS = {
    "power": ("POWER", ["1", "2", "3", "4", "5"], "set_power_select", "mdi:fire"),
    "mode": (
        "MODE",
        ["Manual", "Auto", "Overnight", "Comfort", "Turbo"],
        "set_mode_select",
        "mdi:thermostat-auto",
    ),
    "fan": ("FAN", ["Auto", "1", "2", "3", "4", "5"], "set_fan_select", "mdi:fan"),
}

CONFIG_SCHEMA = HUB_SCHEMA.extend(
    {
        cv.Optional(k): select.select_schema(MczSelect, icon=icon)
        for k, (_, _, _, icon) in SELECTS.items()
    }
)


async def to_code(config):
    hub = await cg.get_variable(config[CONF_MCZ_MAESTRO_BLE_ID])
    for key, (member, options, setter, _) in SELECTS.items():
        if key in config:
            var = await select.new_select(config[key], options=options)
            cg.add(var.set_parent(hub))
            cg.add(var.set_type(getattr(SelectType, member)))
            cg.add(getattr(hub, setter)(var))
