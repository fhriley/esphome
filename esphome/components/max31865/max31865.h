#pragma once

#include "esphome/components/sensor/sensor.h"
#include "esphome/components/spi/spi.h"
#include "esphome/core/component.h"

#ifdef USE_BINARY_SENSOR
#include "esphome/components/binary_sensor/binary_sensor.h"
#endif

#include <cinttypes>

namespace esphome::max31865 {

enum MAX31865RegisterMasks { SPI_WRITE_M = 0x80 };
enum MAX31865Registers {
  CONFIGURATION_REG = 0x00,
  RTD_RESISTANCE_MSB_REG = 0x01,
  RTD_RESISTANCE_LSB_REG = 0x02,
  FAULT_THRESHOLD_H_MSB_REG = 0x03,
  FAULT_THRESHOLD_H_LSB_REG = 0x04,
  FAULT_THRESHOLD_L_MSB_REG = 0x05,
  FAULT_THRESHOLD_L_LSB_REG = 0x06,
  FAULT_STATUS_REG = 0x07,
};
enum MAX31865ConfigFilter {
  FILTER_60HZ = 0,
  FILTER_50HZ = 1,
};

class MAX31865Sensor final : public sensor::Sensor,
                             public Component,
                             public spi::SPIDevice<spi::BIT_ORDER_MSB_FIRST, spi::CLOCK_POLARITY_LOW,
                                                   spi::CLOCK_PHASE_TRAILING, spi::DATA_RATE_4MHZ> {
#ifdef USE_BINARY_SENSOR
  SUB_BINARY_SENSOR(has_fault)
#endif

 public:
  void set_reference_resistance(float reference_resistance) { this->reference_resistance_ = reference_resistance; }
  void set_nominal_resistance(float nominal_resistance) { this->rtd_nominal_resistance_ = nominal_resistance; }
  void set_filter(MAX31865ConfigFilter filter) { this->filter_ = filter; }
  void set_num_rtd_wires(uint8_t rtd_wires) { this->rtd_wires_ = rtd_wires; }
  void set_data_ready_pin(GPIOPin *pin) { this->data_ready_ = pin; }
  void set_update_interval(uint32_t update_interval) { this->update_interval_ = update_interval; }
  /// Worst-case time for one one-shot conversion with the configured filter.
  void set_conversion_time(uint16_t conversion_time_ms) { this->conversion_time_ms_ = conversion_time_ms; }

  void setup() override;
  void loop() override;
  void dump_config() override;

 protected:
  float reference_resistance_;
  float rtd_nominal_resistance_;
  GPIOPin *data_ready_{nullptr};
  uint32_t update_interval_{SCHEDULER_DONT_RUN};
  uint32_t last_fault_check_{0};
  uint16_t conversion_time_ms_{0};
  MAX31865ConfigFilter filter_{FILTER_60HZ};
  uint8_t rtd_wires_{4};
  uint8_t base_config_{0};
  uint8_t faults_{0};

  void configure_();
  void start_measurement_();
  void start_fault_detection_();
  void finish_fault_detection_();
  void read_oneshot_();
  void process_reading_(uint16_t raw);
  bool check_faults_(uint8_t faults);
  void publish_invalid_();

  void write_register_(uint8_t reg, uint8_t value);
  uint8_t read_register_(uint8_t reg);
  uint16_t read_register_16_(uint8_t reg);
  float calc_temperature_(float rtd_ratio);
};

}  // namespace esphome::max31865
