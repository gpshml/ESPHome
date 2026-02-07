import esphome.codegen as cg
import esphome.config_validation as cv
from esphome import pins
from esphome.components import i2c, touchscreen
from esphome.const import CONF_ID, CONF_INTERRUPT_PIN, CONF_ADDRESS, CONF_RESET_PIN

DEPENDENCIES = ["i2c"]
AUTO_LOAD = ["touchscreen"]

print("SPD2010_TOUCH: LOADED FROM GITHUB COMMIT <hash>")

spd2010_ns = cg.esphome_ns.namespace("spd2010_touch")
SPD2010Touch = spd2010_ns.class_(
    "SPD2010Touch",
    touchscreen.Touchscreen,
    i2c.I2CDevice,
)


CONF_POLLING_FALLBACK_MS = "polling_fallback_ms"

CONFIG_SCHEMA = (
    touchscreen.TOUCHSCREEN_SCHEMA.extend(
        {
            cv.GenerateID(): cv.declare_id(SPD2010Touch),
            cv.Optional(CONF_INTERRUPT_PIN): pins.gpio_input_pin_schema,
            cv.Optional(CONF_ADDRESS, default=0x53): cv.hex_int,
            cv.Optional(CONF_RESET_PIN): pins.gpio_output_pin_schema,
            cv.Optional(CONF_POLLING_FALLBACK_MS, default=50): cv.int_range(min=10, max=1000),
        }
    )
    .extend(i2c.i2c_device_schema(0x53))
)

async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await touchscreen.register_touchscreen(var, config)
    await i2c.register_i2c_device(var, config)

    if CONF_INTERRUPT_PIN in config:
        irq = await cg.gpio_pin_expression(config[CONF_INTERRUPT_PIN])
        cg.add(var.set_interrupt_pin(irq))

    if CONF_RESET_PIN in config:
        rst = await cg.gpio_pin_expression(config[CONF_RESET_PIN])
        cg.add(var.set_reset_pin(rst))
        
    cg.add(var.set_polling_fallback_ms(config[CONF_POLLING_FALLBACK_MS]))










