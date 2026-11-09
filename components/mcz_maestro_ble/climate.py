import esphome.codegen as cg
from esphome.components import climate

from . import CONF_MCZ_MAESTRO_BLE_ID, HUB_SCHEMA, mcz_ns

MczClimate = mcz_ns.class_("MczClimate", climate.Climate)

CONFIG_SCHEMA = climate.climate_schema(MczClimate).extend(HUB_SCHEMA)


async def to_code(config):
    hub = await cg.get_variable(config[CONF_MCZ_MAESTRO_BLE_ID])
    var = await climate.new_climate(config)
    cg.add(var.set_parent(hub))
    cg.add(hub.set_climate(var))
