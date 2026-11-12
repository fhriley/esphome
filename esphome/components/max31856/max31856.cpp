#include "max31856.h"

#include "esphome/core/application.h"
#include "esphome/core/hal.h"
#include "esphome/core/helpers.h"
#include "esphome/core/log.h"

namespace esphome::max31856 {

ESPHOME_LOG_TAG(TAG, "max31856");

static constexpr uint32_t READ_TIMEOUT_ID = 0;
static constexpr uint32_t CONFIGURE_TIMEOUT_ID = 1;

// Longest conversion any earlier configuration could still be running when the ESP restarts while the
// chip keeps power: a 50 Hz one-shot of 16 samples (185 + 15 * 40 ms) plus the longest open-circuit test (125 ms).
static constexpr uint32_t MAX_CONVERSION_TIME_MS = 1000;
// An over- or undervoltage fault stops conversions, so DRDY never falls; check the faults this often anyway.
static constexpr uint32_t FAULT_CHECK_INTERVAL_MS = 1000;

enum MAX31856RegisterMasks : uint8_t { SPI_WRITE_M = 0x80 };

enum MAX31856Registers : uint8_t {
  MAX31856_MASK_REG = 0x02,    ///< Fault Mask register
  MAX31856_CJHF_REG = 0x03,    ///< Cold junction High temp fault register
  MAX31856_CJLF_REG = 0x04,    ///< Cold junction Low temp fault register
  MAX31856_LTHFTH_REG = 0x05,  ///< Linearized Temperature High Fault Threshold Register, MSB
  MAX31856_LTHFTL_REG = 0x06,  ///< Linearized Temperature High Fault Threshold Register, LSB
  MAX31856_LTLFTH_REG = 0x07,  ///< Linearized Temperature Low Fault Threshold Register, MSB
  MAX31856_LTLFTL_REG = 0x08,  ///< Linearized Temperature Low Fault Threshold Register, LSB
  MAX31856_CJTO_REG = 0x09,    ///< Cold-Junction Temperature Offset Register
  MAX31856_CJTH_REG = 0x0A,    ///< Cold-Junction Temperature Register, MSB
  MAX31856_CJTL_REG = 0x0B,    ///< Cold-Junction Temperature Register, LSB
  MAX31856_LTCBH_REG = 0x0C,   ///< Linearized TC Temperature, Byte 2
  MAX31856_LTCBM_REG = 0x0D,   ///< Linearized TC Temperature, Byte 1
  MAX31856_LTCBL_REG = 0x0E,   ///< Linearized TC Temperature, Byte 0
  MAX31856_SR_REG = 0x0F,      ///< Fault Status Register

  MAX31856_FAULT_CJRANGE = 0x80,  ///< Fault status Cold Junction Out-of-Range flag
  MAX31856_FAULT_TCRANGE = 0x40,  ///< Fault status Thermocouple Out-of-Range flag
  MAX31856_FAULT_CJHIGH = 0x20,   ///< Fault status Cold-Junction High Fault flag
  MAX31856_FAULT_CJLOW = 0x10,    ///< Fault status Cold-Junction Low Fault flag
  MAX31856_FAULT_TCHIGH = 0x08,   ///< Fault status Thermocouple Temperature High Fault flag
  MAX31856_FAULT_TCLOW = 0x04,    ///< Fault status Thermocouple Temperature Low Fault flag
  MAX31856_FAULT_OVUV = 0x02,     ///< Fault status Overvoltage or Undervoltage Input Fault flag
  MAX31856_FAULT_OPEN = 0x01,     ///< Fault status Thermocouple Open-Circuit Fault flag
};

static constexpr uint8_t MAX31856_CR0_REG = 0x00;
static constexpr uint8_t MAX31856_CR0_AUTOCONVERT = 1 << 7;
static constexpr uint8_t MAX31856_CR0_1SHOT = 1 << 6;
static constexpr uint8_t MAX31856_CR0_OCFAULT01 = 1 << 4;
static constexpr uint8_t MAX31856_CR0_CJ_SENSOR_DISABLE = 1 << 3;
static constexpr uint8_t MAX31856_CR0_FAULT_INT_MODE = 1 << 2;
static constexpr uint8_t MAX31856_CR0_FAULTCLR = 1 << 1;

static constexpr uint8_t MAX31856_CR1_REG = 0x01;

// Based on Adafruit's library: https://github.com/adafruit/Adafruit_MAX31856

static uint8_t samples_per_value_to_int(MAX31856SamplesPerValue value) { return 1 << (value >> 4); }

void MAX31856Sensor::setup() {
  this->spi_setup();
  this->disable_loop();
  if (this->data_ready_ != nullptr) {
    // Data ready is active low
    this->data_ready_->pin_mode(gpio::FLAG_INPUT | gpio::FLAG_PULLUP);
  }

  // The filter and averaging may only change while no conversion runs. The chip keeps converting when only
  // the ESP restarts, so stop it and let the last conversion finish before configuring.
  if ((this->read_register_(MAX31856_CR0_REG) & (MAX31856_CR0_AUTOCONVERT | MAX31856_CR0_1SHOT)) != 0) {
    this->write_register_(MAX31856_CR0_REG, 0);
    this->set_timeout(CONFIGURE_TIMEOUT_ID, MAX_CONVERSION_TIME_MS, [this]() { this->configure_(); });
  } else {
    this->configure_();
  }
}

void MAX31856Sensor::configure_() {
  // Set the thermocouple type and the number of samples to average
  this->write_register_(MAX31856_CR1_REG, static_cast<uint8_t>(this->samples_per_value_) |
                                              (static_cast<uint8_t>(this->thermocouple_type_) & 0x0F));

  // Turn on open circuit faults and set the filter
  this->cr0_ = MAX31856_CR0_OCFAULT01 | static_cast<uint8_t>(this->filter_);

  if (this->data_ready_ != nullptr) {
    this->cr0_ |= MAX31856_CR0_AUTOCONVERT;
    this->last_fault_check_ = millis();
    this->enable_loop();
  } else {
    this->set_interval(this->update_interval_, [this]() { this->start_oneshot_(); });
  }

  this->write_register_(MAX31856_CR0_REG, this->cr0_);
}

void MAX31856Sensor::dump_config() {
  LOG_SENSOR("", "MAX31856", this);
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
                "  Mains Filter: %s\n"
                "  Thermocouple Type: %c\n"
                "  Samples per value: %u\n"
                "  Mode: %s",
                this->filter_ == FILTER_60HZ ? LOG_STR_LITERAL("60 Hz") : LOG_STR_LITERAL("50 Hz"),
                "BEJKNRST"[this->thermocouple_type_ & 0x07], samples_per_value_to_int(this->samples_per_value_),
                this->data_ready_ != nullptr ? LOG_STR_LITERAL("autoconversion") : LOG_STR_LITERAL("polling"));
#ifdef USE_BINARY_SENSOR
  LOG_BINARY_SENSOR("  ", "Has Fault", this->has_fault_binary_sensor_);
#endif
}

// Used in autoconversion mode
void MAX31856Sensor::loop() {
  const uint32_t now = App.get_loop_component_start_time();

  // Data ready is active low
  if (this->data_ready_->digital_read()) {
    if (now - this->last_fault_check_ >= FAULT_CHECK_INTERVAL_MS) {
      this->last_fault_check_ = now;
      this->check_faults_();
    }
    return;
  }

  this->last_fault_check_ = now;
  const bool valid = !this->check_faults_();
  // Reading the temperature also releases DRDY, so read it even when the value is not published
  const uint32_t raw = this->read_register24_(MAX31856_LTCBH_REG);
  if (valid) {
    this->publish_temperature_(raw);
  }
}

// Used in polling mode
void MAX31856Sensor::start_oneshot_() {
  this->write_register_(MAX31856_CR0_REG, this->cr0_ | MAX31856_CR0_1SHOT);
  this->set_timeout(READ_TIMEOUT_ID, this->conversion_time_ms_, [this]() { this->read_oneshot_temperature_(); });
}

void MAX31856Sensor::read_oneshot_temperature_() {
  if (!this->check_faults_()) {
    this->publish_temperature_(this->read_register24_(MAX31856_LTCBH_REG));
  }
}

void MAX31856Sensor::publish_temperature_(uint32_t raw) {
  // bottom 5 bits are unused
  const int32_t temp24 = sign_extend<24>(raw) >> 5;
  const float temp_c = temp24 * 0.0078125f;

  ESP_LOGD(TAG, "Got thermocouple temperature: %.3f°C", temp_c);
  this->publish_state(temp_c);
}

// Returns true when a fault makes the current reading invalid
bool MAX31856Sensor::check_faults_() {
  const uint8_t faults = this->read_register_(MAX31856_SR_REG);
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

  if ((new_faults & MAX31856_FAULT_CJRANGE) != 0) {
    ESP_LOGW(TAG, "Cold Junction Out-of-Range: '%s'", LOG_STR_ARG(this->get_log_name()));
  }
  if ((new_faults & MAX31856_FAULT_TCRANGE) != 0) {
    ESP_LOGW(TAG, "Thermocouple Out-of-Range: '%s'", LOG_STR_ARG(this->get_log_name()));
  }
  if ((new_faults & MAX31856_FAULT_CJHIGH) != 0) {
    ESP_LOGW(TAG, "Cold-Junction High Fault: '%s'", LOG_STR_ARG(this->get_log_name()));
  }
  if ((new_faults & MAX31856_FAULT_CJLOW) != 0) {
    ESP_LOGW(TAG, "Cold-Junction Low Fault: '%s'", LOG_STR_ARG(this->get_log_name()));
  }
  if ((new_faults & MAX31856_FAULT_TCHIGH) != 0) {
    ESP_LOGW(TAG, "Thermocouple Temperature High Fault: '%s'", LOG_STR_ARG(this->get_log_name()));
  }
  if ((new_faults & MAX31856_FAULT_TCLOW) != 0) {
    ESP_LOGW(TAG, "Thermocouple Temperature Low Fault: '%s'", LOG_STR_ARG(this->get_log_name()));
  }
  if ((new_faults & MAX31856_FAULT_OVUV) != 0) {
    ESP_LOGW(TAG, "Overvoltage or Undervoltage Input Fault: '%s'", LOG_STR_ARG(this->get_log_name()));
  }
  if ((new_faults & MAX31856_FAULT_OPEN) != 0) {
    ESP_LOGW(TAG, "Thermocouple Open-Circuit Fault (possibly not connected): '%s'", LOG_STR_ARG(this->get_log_name()));
  }

  // An out-of-range temperature is clamped to the range limit and still published; with an open circuit or
  // an input voltage fault the reading is meaningless or stale.
  return (faults & (MAX31856_FAULT_OPEN | MAX31856_FAULT_OVUV)) != 0;
}

void MAX31856Sensor::write_register_(uint8_t reg, uint8_t value) {
  ESP_LOGVV(TAG, "write_register_ raw reg=0x%02X value=0x%02X", reg, value);
  reg |= SPI_WRITE_M;
  ESP_LOGVV(TAG, "write_register_ masked reg=0x%02X value=0x%02X", reg, value);
  this->enable();
  ESP_LOGVV(TAG, "write_byte reg=0x%02X", reg);
  this->write_byte(reg);
  ESP_LOGVV(TAG, "write_byte value=0x%02X", value);
  this->write_byte(value);
  this->disable();
  ESP_LOGV(TAG, "write_register_ 0x%02X: 0x%02X", reg, value);
}

uint8_t MAX31856Sensor::read_register_(uint8_t reg) {
  ESP_LOGVV(TAG, "read_register_ 0x%02X", reg);
  this->enable();
  ESP_LOGVV(TAG, "write_byte reg=0x%02X", reg);
  this->write_byte(reg);
  const uint8_t value(this->read_byte());
  ESP_LOGVV(TAG, "read_byte value=0x%02X", value);
  this->disable();
  ESP_LOGV(TAG, "read_register_ reg=0x%02X: value=0x%02X", reg, value);
  return value;
}

uint32_t MAX31856Sensor::read_register24_(uint8_t reg) {
  ESP_LOGVV(TAG, "read_register_24_ 0x%02X", reg);
  this->enable();
  ESP_LOGVV(TAG, "write_byte reg=0x%02X", reg);
  this->write_byte(reg);
  const uint8_t msb(this->read_byte());
  ESP_LOGVV(TAG, "read_byte msb=0x%02X", msb);
  const uint8_t mid(this->read_byte());
  ESP_LOGVV(TAG, "read_byte mid=0x%02X", mid);
  const uint8_t lsb(this->read_byte());
  ESP_LOGVV(TAG, "read_byte lsb=0x%02X", lsb);
  this->disable();
  const uint32_t value = encode_uint24(msb, mid, lsb);
  ESP_LOGV(TAG, "read_register_24_ reg=0x%02X: value=0x%06" PRIX32, reg, value);
  return value;
}

}  // namespace esphome::max31856
