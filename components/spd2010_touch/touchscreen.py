import esphome.codegen as cg
import esphome.config_validation as cv
from esphome import pins
from esphome.components import i2c, touchscreen
from esphome.const import CONF_ID, CONF_INTERRUPT_PIN, CONF_ADDRESS, CONF_RESET_PIN

DEPENDENCIES = ["i2c"]
AUTO_LOAD = ["touchscreen"]


CONF_POLLING_FALLBACK_MS = "polling_fallback_ms"
CONF_PCA9554_ID = "pca9554_id"
CONF_RESET_IO = "reset_io"

print("SPD2010_TOUCH: LOADED FROM GITHUB COMMIT <hash>")

spd2010_ns = cg.esphome_ns.namespace("spd2010_touch")
SPD2010Touch = spd2010_ns.class_(
    "SPD2010Touch",
    touchscreen.Touchscreen,
    i2c.I2CDevice,
)

CONFIG_SCHEMA = (
    touchscreen.TOUCHSCREEN_SCHEMA.extend(
        {
            cv.GenerateID(): cv.declare_id(SPD2010Touch),
            cv.Optional(CONF_INTERRUPT_PIN): pins.gpio_input_pin_schema,
            cv.Optional(CONF_ADDRESS, default=0x53): cv.hex_int,
            # NEW: use expander pin for reset instead of a GPIOPin (avoids conflicts)
            cv.Optional(CONF_PCA9554_ID): cv.use_id(pca9554.PCA9554),
            cv.Optional(CONF_RESET_IO): cv.int_range(min=0, max=7),
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

    if CONF_PCA9554_ID in config and CONF_RESET_IO in config:
        exp = await cg.get_variable(config[CONF_PCA9554_ID])
        cg.add(var.set_reset_expander(exp, config[CONF_RESET_IO]))
        
    cg.add(var.set_polling_fallback_ms(config[CONF_POLLING_FALLBACK_MS]))











