#pragma once

#include "esphome/core/component.h"
#include "esphome/components/i2c/i2c.h"
#include "esphome/components/touchscreen/touchscreen.h"
#include "esphome/core/hal.h"
#include "esphome/core/gpio.h"
#include "esphome/components/pca9554/pca9554.h"

namespace esphome {
namespace spd2010_touch {

struct TpStatusHigh {
  uint8_t cpu_run;
  uint8_t tint_low;
  uint8_t tic_in_cpu;
  uint8_t tic_in_bios;
  uint8_t tic_busy;
};

struct TpStatusLow {
  uint8_t pt_exist;
  uint8_t gesture;
  uint8_t aux;
};

struct TpStatus {
  TpStatusLow low{};
  TpStatusHigh high{};
  uint16_t read_len{0};
};

struct TpHdpStatus {
  uint8_t status{0};
  uint16_t next_packet_len{0};
};

struct TouchReport {
  uint8_t id{0};
  uint16_t x{0};
  uint16_t y{0};
  uint8_t weight{0};
};

struct TouchFrame {
  TouchReport rpt[5];
  uint8_t touch_num{0};
};

class SPD2010Touch : public touchscreen::Touchscreen,
                     public i2c::I2CDevice {
 public:
  void set_interrupt_pin(InternalGPIOPin *pin) { this->irq_pin_ = pin; }
  void set_polling_fallback_ms(uint16_t ms) { this->polling_fallback_ms_ = ms; }

  // NEW:
  void set_reset_expander(pca9554::PCA9554 *exp, uint8_t pin) {
    this->reset_expander_ = exp;
    this->reset_expander_pin_ = pin;
  }
  // Call this AFTER the display init sequence (0x11 -> delay -> 0x29 -> delay)
  void notify_display_ready();

  void setup() override;
  void loop() override;
  void dump_config() override;

 protected:
  void update_touches() override;
  // NEW helpers
  void pulse_shared_reset_();
  bool probe_ack_();
  void log_i2c_probe_scan_();

  // Deferred init
  void try_init_();

  // SPD2010 protocol helpers (ported from your driver)
  bool read16_(uint16_t reg, uint8_t *data, size_t len);
  bool write16_(uint16_t reg, const uint8_t *data, size_t len);

  void write_tp_point_mode_cmd_();
  void write_tp_start_cmd_();
  void write_tp_cpu_start_cmd_();
  void write_tp_clear_int_cmd_();
  void read_tp_status_length_(TpStatus *st);
  void read_tp_hdp_(const TpStatus &st, TouchFrame *frame);
  void read_tp_hdp_status_(TpHdpStatus *hs);
  void read_hdp_remain_data_(const TpHdpStatus &hs);

  void tp_read_data_(TouchFrame *frame);

  InternalGPIOPin *irq_pin_{nullptr};
  pca9554::PCA9554 *reset_expander_{nullptr};
  uint8_t reset_expander_pin_{255};

  volatile bool irq_fired_{false};
  uint32_t last_poll_ms_{0};
  uint16_t polling_fallback_ms_{50};

  volatile bool irq_fired_{false};
  uint32_t last_poll_ms_{0};
  uint16_t polling_fallback_ms_{50};

  // New state for deferred init
  bool display_ready_{false};
  bool initialised_{false};
  uint32_t next_probe_ms_{0};
  uint32_t boot_ms_{0};
  uint8_t init_attempts_{0};
  uint32_t next_init_try_ms_{0};

  static void IRAM_ATTR gpio_isr_(SPD2010Touch *self) { self->irq_fired_ = true; }
};

}  // namespace spd2010_touch
}  // namespace esphome



