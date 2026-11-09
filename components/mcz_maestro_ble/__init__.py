"""ESPHome component for MCZ Maestro+ pellet stoves over the display's own BLE.

Port of the PlatformIO firmware in src/; the wire protocol lives in mcz_proto.h,
shared by both builds.
"""

import esphome.codegen as cg
from esphome.components import ble_client, time as time_
import esphome.config_validation as cv
from esphome.const import CONF_ID, CONF_TIME_ID

CODEOWNERS = ["@muka"]
DEPENDENCIES = ["ble_client"]
MULTI_CONF = True

CONF_MCZ_MAESTRO_BLE_ID = "mcz_maestro_ble_id"
CONF_POLL_INTERVAL = "poll_interval"
CONF_PAIR_ON_CONNECT = "pair_on_connect"

mcz_ns = cg.esphome_ns.namespace("mcz_maestro_ble")
MczMaestroBle = mcz_ns.class_("MczMaestroBle", cg.Component, ble_client.BLEClientNode)

CONFIG_SCHEMA = (
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(MczMaestroBle),
            cv.Optional(CONF_POLL_INTERVAL, default="2500ms"): cv.All(
                cv.positive_time_period_milliseconds,
                cv.Range(min=cv.TimePeriod(milliseconds=1000)),
            ),
            # Default follows the PlatformIO build: answer the display's security
            # request ("just works" bonding) but do not start pairing ourselves.
            cv.Optional(CONF_PAIR_ON_CONNECT, default=False): cv.boolean,
            cv.Optional(CONF_TIME_ID): cv.use_id(time_.RealTimeClock),
        }
    )
    .extend(ble_client.BLE_CLIENT_SCHEMA)
    .extend(cv.COMPONENT_SCHEMA)
)

HUB_SCHEMA = cv.Schema(
    {cv.GenerateID(CONF_MCZ_MAESTRO_BLE_ID): cv.use_id(MczMaestroBle)}
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await ble_client.register_ble_node(var, config)
    cg.add(var.set_poll_interval(config[CONF_POLL_INTERVAL]))
    cg.add(var.set_pair_on_connect(config[CONF_PAIR_ON_CONNECT]))
    if CONF_TIME_ID in config:
        cg.add_define("MCZ_USE_TIME")
        t = await cg.get_variable(config[CONF_TIME_ID])
        cg.add(var.set_time(t))
