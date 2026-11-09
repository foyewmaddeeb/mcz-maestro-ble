import esphome.codegen as cg
from esphome.components import switch
import esphome.config_validation as cv

from . import CONF_MCZ_MAESTRO_BLE_ID, HUB_SCHEMA, mcz_ns

MczSwitch = mcz_ns.class_("MczSwitch", switch.Switch)

CONFIG_SCHEMA = HUB_SCHEMA.extend(
    {cv.Optional("silent"): switch.switch_schema(MczSwitch, icon="mdi:volume-off")}
)


async def to_code(config):
    hub = await cg.get_variable(config[CONF_MCZ_MAESTRO_BLE_ID])
    if "silent" in config:
        var = await switch.new_switch(config["silent"])
        cg.add(var.set_parent(hub))
        cg.add(hub.set_silent_switch(var))
