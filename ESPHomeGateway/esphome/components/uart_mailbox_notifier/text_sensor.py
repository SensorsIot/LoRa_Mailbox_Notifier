import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import text_sensor, uart
from esphome.const import CONF_ID

uart_mailbox_ns = cg.esphome_ns.namespace("uart_mailbox_notifier")
UartMailboxNotifier = uart_mailbox_ns.class_(
    "Uart_Mailbox_Notifier",
    cg.Component,
    text_sensor.TextSensor,
    uart.UARTDevice,
)

CONFIG_SCHEMA = (
    text_sensor.text_sensor_schema()
    .extend(
        {
            cv.GenerateID(): cv.declare_id(UartMailboxNotifier),
        }
    )
    .extend(uart.UART_DEVICE_SCHEMA)
)

async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await text_sensor.register_text_sensor(var, config)
    await uart.register_uart_device(var, config)
