# © Copyright 2025 Stuart Parmenter
# SPDX-License-Identifier: MIT

from esphome import codegen as cg
import esphome.config_validation as cv
from esphome.const import CONF_ID, CONF_PORT

DEPENDENCIES = ["network", "lvgl"]

ddp_ns = cg.esphome_ns.namespace("ddp_stream")
DdpStream = ddp_ns.class_("DdpStream", cg.Component)

CONF_STREAMS = "streams"

# NEW global options
CONF_PRESENT_ON_TIMEOUT = "present_on_timeout"
CONF_TIMEOUT_COVERAGE   = "timeout_coverage"
CONF_TIMEOUT_FACTOR     = "timeout_factor"
CONF_DEFAULT_FPS        = "default_fps"
CONF_ADAPT_FPS          = "adapt_fps"

STREAM_SCHEMA = cv.Schema({
    cv.Required("id"): cv.int_range(min=0, max=255),
    cv.Required("canvas_id"): cv.string,   # YAML id of the LVGL canvas
    cv.Optional("width", default=-1): cv.int_,
    cv.Optional("height", default=-1): cv.int_,
})

CONFIG_SCHEMA = cv.Schema({
    cv.GenerateID(): cv.declare_id(DdpStream),
    cv.Optional(CONF_PORT, default=4048): cv.port,
    cv.Optional(CONF_STREAMS, default=[]): cv.ensure_list(STREAM_SCHEMA),

    # ---- NEW (all optional) ----
    cv.Optional(CONF_PRESENT_ON_TIMEOUT, default=False): cv.boolean,
    cv.Optional(CONF_TIMEOUT_COVERAGE,   default=0.90): cv.float_range(min=0.0, max=1.0),
    cv.Optional(CONF_TIMEOUT_FACTOR,     default=1.20): cv.float_range(min=1.0, max=3.0),
    cv.Optional(CONF_DEFAULT_FPS,        default=25.0): cv.float_range(min=1.0, max=240.0),
    cv.Optional(CONF_ADAPT_FPS,          default=True): cv.boolean,
})

async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    cg.add(var.set_port(config[CONF_PORT]))

    # NEW: apply global options
    cg.add(var.set_present_on_timeout(config[CONF_PRESENT_ON_TIMEOUT]))
    cg.add(var.set_timeout_coverage(config[CONF_TIMEOUT_COVERAGE]))
    cg.add(var.set_timeout_factor(config[CONF_TIMEOUT_FACTOR]))
    cg.add(var.set_default_fps(config[CONF_DEFAULT_FPS]))
    cg.add(var.set_adapt_fps(config[CONF_ADAPT_FPS]))

    # Defer binding: hand a pointer-getter lambda that will be called after LVGL creates widgets
    for s in config.get(CONF_STREAMS, []):
        canvas = s["canvas_id"]
        getter = cg.RawExpression('[]() -> lv_obj_t* { return &id(%s); }' % canvas)
        cg.add(var.add_stream_binding(s["id"], getter, s.get("width", -1), s.get("height", -1)))
