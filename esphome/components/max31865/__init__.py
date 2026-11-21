import esphome.codegen as cg
from esphome.components import sensor, spi

DOMAIN = "max31865"

CONF_MAX31865_ID = "max31865_id"

max31865_ns = cg.esphome_ns.namespace("max31865")
MAX31865Sensor = max31865_ns.class_(
    "MAX31865Sensor", sensor.Sensor, cg.Component, spi.SPIDevice
)
