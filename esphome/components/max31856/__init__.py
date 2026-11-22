import esphome.codegen as cg
from esphome.components import sensor, spi

DOMAIN = "max31856"

CONF_MAX31856_ID = "max31856_id"

max31856_ns = cg.esphome_ns.namespace("max31856")
MAX31856Sensor = max31856_ns.class_(
    "MAX31856Sensor", sensor.Sensor, cg.Component, spi.SPIDevice
)
