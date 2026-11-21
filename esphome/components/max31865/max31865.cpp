#include "max31865.h"

#include "esphome/core/application.h"
#include "esphome/core/hal.h"
#include "esphome/core/helpers.h"
#include "esphome/core/log.h"
#include <cmath>

namespace esphome::max31865 {

ESPHOME_LOG_TAG(TAG, "max31865");

static constexpr uint32_t SEQUENCE_TIMEOUT_ID = 0;
static constexpr uint32_t CONFIGURE_TIMEOUT_ID = 1;

// Configuration register bits
static constexpr uint8_t CONFIG_BIAS = 1 << 7;
static constexpr uint8_t CONFIG_AUTO_CONVERT = 1 << 6;
static constexpr uint8_t CONFIG_1SHOT = 1 << 5;
static constexpr uint8_t CONFIG_3WIRE = 1 << 4;
static constexpr uint8_t CONFIG_FAULT_CYCLE_MASK = 0b11 << 2;
static constexpr uint8_t CONFIG_FAULT_CYCLE_AUTO = 0b01 << 2;
static constexpr uint8_t CONFIG_FAULT_CLEAR = 1 << 1;
static constexpr uint8_t CONFIG_FILTER_50HZ = 1 << 0;

// Fault status register bits; D1 and D0 are unused
static constexpr uint8_t FAULT_RTD_HIGH = 1 << 7;
static constexpr uint8_t FAULT_RTD_LOW = 1 << 6;
static constexpr uint8_t FAULT_REFIN_HIGH = 1 << 5;
static constexpr uint8_t FAULT_REFIN_LOW_FORCE_OPEN = 1 << 4;
static constexpr uint8_t FAULT_RTDIN_LOW_FORCE_OPEN = 1 << 3;
static constexpr uint8_t FAULT_OVUV = 1 << 2;
static constexpr uint8_t FAULT_MASK = 0b11111100;

// After V_BIAS turns on, the input filter needs 10.5 time constants plus 1 ms. The datasheet's 10 ms
// bias startup time covers the largest recommended network (10 kΩ reference, 0.1 µF).
static constexpr uint32_t BIAS_SETTLE_MS = 11;
// The automatic fault detection cycle takes at most 600 µs; the input filter then needs 5 time constants
// plus 1 ms before the ADC starts.
static constexpr uint32_t FAULT_DETECTION_MS = 7;
// Longest conversion an earlier configuration could still be running when the ESP restarts while the chip keeps
// power: a 50 Hz one-shot conversion takes at most 66 ms.
static constexpr uint32_t MAX_CONVERSION_TIME_MS = 70;
// An over- or undervoltage fault stops conversions, so DRDY never falls; check the faults this often anyway.
static constexpr uint32_t FAULT_CHECK_INTERVAL_MS = 1000;
// Retry delay in autoconversion mode when the fault detection cycle does not finish
static constexpr uint32_t CONFIGURE_RETRY_MS = 1000;

void MAX31865Sensor::setup() {
  this->spi_setup();
  this->disable_loop();
  if (this->data_ready_ != nullptr) {
    this->data_ready_->setup();
  }

  this->base_config_ = this->filter_ == FILTER_50HZ ? CONFIG_FILTER_50HZ : 0;
  if (this->rtd_wires_ == 3) {
    this->base_config_ |= CONFIG_3WIRE;
  }

  // The filter may only change while no conversion runs. The chip keeps converting when only the ESP
  // restarts, so stop it and let the last conversion finish before configuring.
  const uint8_t config = this->read_register_(CONFIGURATION_REG);
  if ((config & (CONFIG_AUTO_CONVERT | CONFIG_1SHOT | CONFIG_FAULT_CYCLE_MASK)) != 0) {
    this->write_register_(CONFIGURATION_REG, config & CONFIG_FILTER_50HZ);
    this->set_timeout(CONFIGURE_TIMEOUT_ID, MAX_CONVERSION_TIME_MS, [this]() { this->configure_(); });
  } else {
    this->configure_();
  }
}

void MAX31865Sensor::configure_() {
  this->write_register_(CONFIGURATION_REG, this->base_config_ | CONFIG_FAULT_CLEAR);
  if (!this->setup_check((this->read_register_(CONFIGURATION_REG) & ~CONFIG_FAULT_CLEAR) == this->base_config_))
    return;

  if (this->data_ready_ != nullptr) {
    // Check the wiring once, then convert continuously
    this->start_measurement_();
  } else if (this->update_interval_ != SCHEDULER_DONT_RUN) {
    this->set_interval(this->update_interval_, [this]() { this->start_measurement_(); });
    this->start_measurement_();
  }
}

void MAX31865Sensor::dump_config() {
  LOG_SENSOR("", "MAX31865", this);
  LOG_PIN("  CS Pin: ", this->cs_);
  if (this->data_ready_ != nullptr) {
    LOG_PIN("  Data Ready Pin: ", this->data_ready_);
  } else if (this->update_interval_ == SCHEDULER_DONT_RUN) {
    ESP_LOGCONFIG(TAG, "  Update Interval: never");
  } else {
    ESP_LOGCONFIG(TAG, "  Update Interval: %" PRIu32 ".%03" PRIu32 "s", this->update_interval_ / 1000,
                  this->update_interval_ % 1000);
  }
  ESP_LOGCONFIG(TAG,
                "  Reference Resistance: %.2fΩ\n"
                "  RTD: %u-wire %.2fΩ\n"
                "  Mains Filter: %s\n"
                "  Mode: %s",
                this->reference_resistance_, this->rtd_wires_, this->rtd_nominal_resistance_,
                this->filter_ == FILTER_60HZ ? LOG_STR_LITERAL("60 Hz") : LOG_STR_LITERAL("50 Hz"),
                this->data_ready_ != nullptr ? LOG_STR_LITERAL("autoconversion") : LOG_STR_LITERAL("polling"));
#ifdef USE_BINARY_SENSOR
  LOG_BINARY_SENSOR("  ", "Has Fault", this->has_fault_binary_sensor_);
#endif
}

// Used in autoconversion mode
void MAX31865Sensor::loop() {
  const uint32_t now = App.get_loop_component_start_time();

  // Data ready is active low
  if (this->data_ready_->digital_read()) {
    if (now - this->last_fault_check_ >= FAULT_CHECK_INTERVAL_MS) {
      this->last_fault_check_ = now;
      const uint8_t faults = this->read_register_(FAULT_STATUS_REG) & FAULT_MASK;
      if (faults != 0) {
        this->write_register_(CONFIGURATION_REG,
                              this->base_config_ | CONFIG_BIAS | CONFIG_AUTO_CONVERT | CONFIG_FAULT_CLEAR);
        this->check_faults_(faults);
        this->publish_invalid_();
      }
    }
    return;
  }

  this->last_fault_check_ = now;
  // Reading the result also releases DRDY
  this->process_reading_(this->read_register_16_(RTD_RESISTANCE_MSB_REG));
}

// Turns on V_BIAS and starts the sequence: fault detection, then a one-shot conversion or autoconversion
void MAX31865Sensor::start_measurement_() {
  // The fault status latches, so clear it to see only the faults of this measurement
  this->write_register_(CONFIGURATION_REG, this->base_config_ | CONFIG_BIAS | CONFIG_FAULT_CLEAR);
  this->set_timeout(SEQUENCE_TIMEOUT_ID, BIAS_SETTLE_MS, [this]() { this->start_fault_detection_(); });
}

void MAX31865Sensor::start_fault_detection_() {
  this->write_register_(CONFIGURATION_REG, this->base_config_ | CONFIG_BIAS | CONFIG_FAULT_CYCLE_AUTO);
  this->set_timeout(SEQUENCE_TIMEOUT_ID, FAULT_DETECTION_MS, [this]() { this->finish_fault_detection_(); });
}

void MAX31865Sensor::finish_fault_detection_() {
  const uint8_t config = this->read_register_(CONFIGURATION_REG);
  if ((config & CONFIG_FAULT_CYCLE_MASK) != 0) {
    ESP_LOGW(TAG, "Fault detection did not finish (0x%02X): '%s'", config, LOG_STR_ARG(this->get_log_name()));
    this->write_register_(CONFIGURATION_REG, this->base_config_);
    this->status_set_warning();
    this->publish_invalid_();
    if (this->data_ready_ != nullptr) {
      this->set_timeout(CONFIGURE_TIMEOUT_ID, CONFIGURE_RETRY_MS, [this]() { this->start_measurement_(); });
    }
    return;
  }

  // Faults found by the cycle stay latched and are reported with the first result
  if (this->data_ready_ != nullptr) {
    // The first conversion takes as long as a one-shot conversion
    this->write_register_(CONFIGURATION_REG, this->base_config_ | CONFIG_BIAS | CONFIG_AUTO_CONVERT);
    this->last_fault_check_ = millis();
    this->enable_loop();
  } else {
    this->write_register_(CONFIGURATION_REG, this->base_config_ | CONFIG_BIAS | CONFIG_1SHOT);
    this->set_timeout(SEQUENCE_TIMEOUT_ID, this->conversion_time_ms_, [this]() { this->read_oneshot_(); });
  }
}

// Used in polling mode
void MAX31865Sensor::read_oneshot_() {
  const uint16_t raw = this->read_register_16_(RTD_RESISTANCE_MSB_REG);
  // Turn off V_BIAS until the next measurement to save power and avoid self-heating
  this->write_register_(CONFIGURATION_REG, this->base_config_);
  this->process_reading_(raw);
}

void MAX31865Sensor::process_reading_(uint16_t raw) {
  const uint8_t faults = this->read_register_(FAULT_STATUS_REG) & FAULT_MASK;
  if (faults != 0 && this->data_ready_ != nullptr) {
    // Polling mode clears the faults when the next measurement starts
    this->write_register_(CONFIGURATION_REG,
                          this->base_config_ | CONFIG_BIAS | CONFIG_AUTO_CONVERT | CONFIG_FAULT_CLEAR);
  }

  // A zero code always trips the low threshold fault, so zero with no fault means the bus read nothing
  if (raw == 0 && faults == 0) {
    ESP_LOGW(TAG, ESP_LOG_MSG_COMM_FAIL_FOR, LOG_STR_ARG(this->get_log_name()));
    this->status_set_warning();
    this->publish_invalid_();
    return;
  }

  if (this->check_faults_(faults)) {
    this->publish_invalid_();
    return;
  }

  // D0 is the fault bit; the code is the 15-bit ratio of RTD to reference resistance
  const float rtd_ratio = static_cast<float>(raw >> 1) / static_cast<float>(1 << 15);
  const float temperature = this->calc_temperature_(rtd_ratio);
  ESP_LOGV(TAG, "RTD read complete. %.5f (ratio) * %.1fΩ (reference) = %.2fΩ --> %.2f°C", rtd_ratio,
           this->reference_resistance_, this->reference_resistance_ * rtd_ratio, temperature);
  this->publish_state(temperature);
}

// Returns true when a fault makes the reading invalid
bool MAX31865Sensor::check_faults_(uint8_t faults) {
  const uint8_t new_faults = faults & ~this->faults_;
  const bool had_faults = this->faults_ != 0;
  this->faults_ = faults;

#ifdef USE_BINARY_SENSOR
  if (this->has_fault_binary_sensor_ != nullptr) {
    this->has_fault_binary_sensor_->publish_state(faults != 0);
  }
#endif

  if (faults == 0) {
    if (had_faults) {
      ESP_LOGI(TAG, "Faults cleared: '%s'", LOG_STR_ARG(this->get_log_name()));
    }
    this->status_clear_warning();
    return false;
  }

  this->status_set_warning();

  // With the power-on thresholds, only a full-scale code (open RTD) or a zero code (shorted RTD) trips these
  if ((new_faults & FAULT_RTD_HIGH) != 0) {
    ESP_LOGW(TAG, "RTD High Threshold (possibly open RTD): '%s'", LOG_STR_ARG(this->get_log_name()));
  }
  if ((new_faults & FAULT_RTD_LOW) != 0) {
    ESP_LOGW(TAG, "RTD Low Threshold (possibly shorted RTD): '%s'", LOG_STR_ARG(this->get_log_name()));
  }
  if ((new_faults & FAULT_REFIN_HIGH) != 0) {
    ESP_LOGW(TAG, "REFIN- > 0.85 x V_BIAS (possibly open RTD or wire): '%s'", LOG_STR_ARG(this->get_log_name()));
  }
  if ((new_faults & FAULT_REFIN_LOW_FORCE_OPEN) != 0) {
    ESP_LOGW(TAG, "REFIN- < 0.85 x V_BIAS (FORCE- open): '%s'", LOG_STR_ARG(this->get_log_name()));
  }
  if ((new_faults & FAULT_RTDIN_LOW_FORCE_OPEN) != 0) {
    ESP_LOGW(TAG, "RTDIN- < 0.85 x V_BIAS (FORCE- open): '%s'", LOG_STR_ARG(this->get_log_name()));
  }
  if ((new_faults & FAULT_OVUV) != 0) {
    ESP_LOGW(TAG, "Overvoltage or Undervoltage Input Fault: '%s'", LOG_STR_ARG(this->get_log_name()));
  }
  return true;
}

// Publishes NAN once when the reading becomes invalid, not on every conversion
void MAX31865Sensor::publish_invalid_() {
  if (!std::isnan(this->state)) {
    this->publish_state(NAN);
  }
}

void MAX31865Sensor::write_register_(uint8_t reg, uint8_t value) {
  this->enable();
  this->write_byte(reg | SPI_WRITE_M);
  this->write_byte(value);
  this->disable();
  ESP_LOGVV(TAG, "write_register_ 0x%02X: 0x%02X", reg, value);
}

uint8_t MAX31865Sensor::read_register_(uint8_t reg) {
  this->enable();
  this->write_byte(reg);
  const uint8_t value(this->read_byte());
  this->disable();
  ESP_LOGVV(TAG, "read_register_ 0x%02X: 0x%02X", reg, value);
  return value;
}

uint16_t MAX31865Sensor::read_register_16_(uint8_t reg) {
  this->enable();
  this->write_byte(reg);
  const uint8_t msb(this->read_byte());
  const uint8_t lsb(this->read_byte());
  this->disable();
  const uint16_t value = encode_uint16(msb, lsb);
  ESP_LOGVV(TAG, "read_register_16_ 0x%02X: 0x%04X", reg, value);
  return value;
}

float MAX31865Sensor::calc_temperature_(float rtd_ratio) {
  // Based loosely on Adafruit's library: https://github.com/adafruit/Adafruit_MAX31865
  // Mainly based on formulas provided by Analog:
  // http://www.analog.com/media/en/technical-documentation/application-notes/AN709_0.pdf

  const float a = 3.9083e-3;
  const float b = -5.775e-7;
  const float z1 = -a;
  const float z2 = a * a - 4 * b;
  const float z3 = 4 * b / this->rtd_nominal_resistance_;
  const float z4 = 2 * b;

  float rtd_resistance = rtd_ratio * this->reference_resistance_;

  // ≥ 0°C Formula
  const float pos_temp = (z1 + std::sqrt(z2 + (z3 * rtd_resistance))) / z4;
  if (pos_temp >= 0) {
    return pos_temp;
  }

  // < 0°C Formula
  if (this->rtd_nominal_resistance_ != 100) {
    // Normalize RTD to 100Ω
    rtd_resistance /= this->rtd_nominal_resistance_;
    rtd_resistance *= 100;
  }
  float rpoly = rtd_resistance;
  float neg_temp = -242.02f;
  neg_temp += 2.2228f * rpoly;
  rpoly *= rtd_resistance;  // square
  neg_temp += 2.5859e-3f * rpoly;
  rpoly *= rtd_resistance;  // ^3
  neg_temp -= 4.8260e-6f * rpoly;
  rpoly *= rtd_resistance;  // ^4
  neg_temp -= 2.8183e-8f * rpoly;
  rpoly *= rtd_resistance;  // ^5
  neg_temp += 1.5243e-10f * rpoly;
  return neg_temp;
}

}  // namespace esphome::max31865
