#import logging
import esphome.config_validation as cv
import esphome.codegen as cg
from esphome import automation, pins
from esphome.components import sensor, uart, time, text_sensor, binary_sensor, output
from esphome.automation import maybe_simple_id
from esphome.const import (
    CONF_ID,
    CONF_PIN,
    CONF_UART_ID,
    CONF_TIME_ID,
    CONF_CURRENT,
    CONF_DATA,
    STATE_CLASS_MEASUREMENT,
    DEVICE_CLASS_TEMPERATURE,
    DEVICE_CLASS_VOLTAGE,
    ICON_THERMOMETER,
    UNIT_CELSIUS,
)

#_LOGGER = logging.getLogger(__name__)

CODEOWNERS = ["@Brokly"]
DEPENDENCIES = ["sensor", "time", "uart"]
AUTO_LOAD = ["binary_sensor", "text_sensor", "output"]

CONF_TEMPERATURE = "temperature_"
CONF_DALLAS_ADDRESS = "ds_address_"
ICON_DALLAS_SERIAL = "mdi:numeric"
CONF_OUTPUT = "output_"
ICON_OUTPUT = "mdi:electric-switch"
CONF_INPUT = "input_"
ICON_INPUT = "mdi:gauge"
CONF_BATTERY = "rtc_battery_voltage"
ICON_BATTERY = "mdi:battery-clock-outline"
CONF_FIRM_VERSION = "firmware_version"
ICON_FIRM_VERSION = "mdi:select-inverse"
CONF_CONNECT_STATUS = "connect_status"
ICON_CONNECT_STATUS = "mdi:lan-disconnect"
CONF_SIGNAL_LED_PIN = "signal_led_pin"

nmbm8036_ns = cg.esphome_ns.namespace("nmbm8036")
NMBM8036 = nmbm8036_ns.class_("NMBM8036", time.RealTimeClock)
WriteAction = nmbm8036_ns.class_("WriteAction", automation.Action)
ReadAction = nmbm8036_ns.class_("ReadAction", automation.Action)

def output_info(config):
#    _LOGGER.info(config)
    return config

initParams = {
     cv.GenerateID(): cv.declare_id(NMBM8036),
     # напряжение батарейки
     cv.Optional(CONF_BATTERY): sensor.sensor_schema(
        unit_of_measurement="V",
        icon=ICON_BATTERY,
        accuracy_decimals=2,
        device_class=DEVICE_CLASS_VOLTAGE,
        state_class=STATE_CLASS_MEASUREMENT,
     ),
     # версия прошивки устройства
     cv.Optional(CONF_FIRM_VERSION): text_sensor.text_sensor_schema(
        icon=ICON_FIRM_VERSION,
     ),
     # статус соединения с nm8036
     cv.Optional(CONF_CONNECT_STATUS): text_sensor.text_sensor_schema(
        icon=ICON_CONNECT_STATUS,
     ),
     # нога светодиода индикации связи
     cv.Optional(CONF_PIN): pins.gpio_output_pin_schema,
}
# adding termosensors
for number in range(32):
    initParams[cv.Optional(CONF_TEMPERATURE+str(number))] = sensor.sensor_schema(
        device_class=DEVICE_CLASS_TEMPERATURE,
        unit_of_measurement=UNIT_CELSIUS,
        accuracy_decimals=2,
        state_class=STATE_CLASS_MEASUREMENT,
        icon=ICON_THERMOMETER,
    )
    initParams[cv.Optional(CONF_DALLAS_ADDRESS+str(number))] = text_sensor.text_sensor_schema(
        icon=ICON_DALLAS_SERIAL,
    )
#adding output sensors 
for number in range(12):
    initParams[cv.Optional(CONF_OUTPUT+str(number))] = binary_sensor.binary_sensor_schema(
        icon=ICON_OUTPUT,
    )
#adding input sensors 
for number in range(4):
    initParams[cv.Optional(CONF_INPUT+str(number))] = sensor.sensor_schema(
        accuracy_decimals=0,
        state_class=STATE_CLASS_MEASUREMENT,
        icon=ICON_INPUT,
    )
    
CONFIG_SCHEMA = cv.All(sensor.SENSOR_SCHEMA.extend(initParams).extend(time.TIME_SCHEMA).extend(uart.UART_DEVICE_SCHEMA).extend(cv.COMPONENT_SCHEMA), output_info)

@automation.register_action(
    "nmbm8036.write_time",
    WriteAction,
    cv.Schema(
        {
            cv.GenerateID(): cv.use_id(NMBM8036),
        }
    ),
)
async def nmbm8036_write_time_to_code(config, action_id, template_arg, args):
    var = cg.new_Pvariable(action_id, template_arg)
    await cg.register_parented(var, config[CONF_ID])
    return var

@automation.register_action(
    "nmbm8036.read_time",
    ReadAction,
    automation.maybe_simple_id(
        {
            cv.GenerateID(): cv.use_id(NMBM8036),
        }
    ),
)
async def nmbm8036_read_time_to_code(config, action_id, template_arg, args):
    var = cg.new_Pvariable(action_id, template_arg)
    await cg.register_parented(var, config[CONF_ID])
    return var

async def to_code(config):
    #_LOGGER.info("--------------")
    #_LOGGER.info(config)
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    #uart 
    parent = await cg.get_variable(config[CONF_UART_ID])
    cg.add(var.initUart(parent))
    #time
    await time.register_time(var, config)
    # temperature sensors
    for number in range(32):
        if (CONF_TEMPERATURE+str(number)) in config:
            conf = config[CONF_TEMPERATURE+str(number)]
            sens = await sensor.new_sensor(conf)
            cg.add(var.set_temperature_sensor(sens,number))
        if (CONF_DALLAS_ADDRESS+str(number)) in config:
            conf = config[CONF_DALLAS_ADDRESS+str(number)]
            sens = await text_sensor.new_text_sensor(conf)
            cg.add(var.set_dallas_sn_sensor(sens,number))
    # output sensors
    for number in range(12):
        if (CONF_OUTPUT+str(number)) in config:
            conf = config[CONF_OUTPUT+str(number)]
            sens = await binary_sensor.new_binary_sensor(conf)
            cg.add(var.set_output_sensor(sens,number))
    # input sensors
    for number in range(4):
        if (CONF_INPUT+str(number)) in config:
            conf = config[CONF_INPUT+str(number)]
            sens = await sensor.new_sensor(conf)
            cg.add(var.set_adc_sensor(sens,number))
    # батарейка
    if (CONF_BATTERY) in config:
        conf = config[CONF_BATTERY]
        sens = await sensor.new_sensor(conf)
        cg.add(var.set_batt_sensor(sens))
    # прошивка
    if (CONF_FIRM_VERSION) in config:
        conf = config[CONF_FIRM_VERSION]
        sens = await text_sensor.new_text_sensor(conf)
        cg.add(var.set_vers_sensor(sens))
    # connect status
    if (CONF_CONNECT_STATUS) in config:
        conf = config[CONF_CONNECT_STATUS]
        sens = await text_sensor.new_text_sensor(conf)
        cg.add(var.set_err_sensor(sens))
    #нога светодиода сигнализации
    if (CONF_PIN) in config:
        conf=config[CONF_PIN]
        pin = await cg.gpio_pin_expression(conf)
        cg.add(var.set_pin(pin))
