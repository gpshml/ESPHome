#include "spd2010_touch.h"
#include "esphome/core/log.h"

namespace esphome {
namespace spd2010_touch {

static const char *const TAG = "spd2010_touch";

// Key regs from your driver
static constexpr uint16_t REG_CLEAR_INT   = 0x0200; // write 0x01 0x00
static constexpr uint16_t REG_CPU_START   = 0x0400; // write 0x01 0x00
static constexpr uint16_t REG_TOUCH_START = 0x4600; // write 0x00 0x00
static constexpr uint16_t REG_POINT_MODE  = 0x5000; // write 0x00 0x00
static constexpr uint16_t REG_STATUS_LEN  = 0x2000; // read 4 bytes
static constexpr uint16_t REG_HDP_READ    = 0x0003; // read read_len bytes
static constexpr uint16_t REG_HDP_STATUS  = 0xFC02; // read 8 bytes

void SPD2010Touch::setup() {
    ESP_LOGCONFIG(TAG, "Setting up SPD2010 touch...");

    // ---- TP_RST pulse (matches original driver) ----
    if (this->reset_pin_ != nullptr) {
      ESP_LOGCONFIG(TAG, "Resetting SPD2010 touch controller...");
      this->reset_pin_->setup();
      this->reset_pin_->digital_write(false);
      delay(50);
      this->reset_pin_->digital_write(true);
      delay(50);
    } else {
      ESP_LOGW(TAG, "No reset_pin configured for SPD2010 touch");
    }

  this->write_tp_cpu_start_cmd_();
  this->write_tp_point_mode_cmd_();
  this->write_tp_start_cmd_();
  this->write_tp_clear_int_cmd_();

  // Optional IRQ pin
  if (this->irq_pin_ != nullptr) {
    this->irq_pin_->setup();
    // Use Touchscreen base helper to attach interrupt
    // this->attach_interrupt_(this->irq_pin_, gpio::INTERRUPT_FALLING_EDGE);
    // Also set a lightweight flag so we can skip I2C reads when idle
    this->irq_pin_->attach_interrupt(SPD2010Touch::gpio_isr_, this, gpio::INTERRUPT_FALLING_EDGE);
  }
}

void SPD2010Touch::dump_config() {
  ESP_LOGCONFIG(TAG, "SPD2010 Touch:");
  LOG_I2C_DEVICE(this);
  LOG_PIN("  Interrupt Pin: ", this->irq_pin_);
  ESP_LOGCONFIG(TAG, "  Polling fallback: %ums", this->polling_fallback_ms_);
}

void SPD2010Touch::loop() {
  // If IRQ configured: read only when fired
  if (this->irq_pin_ != nullptr) {
    if (!this->irq_fired_) return;
    this->irq_fired_ = false;
    this->update_touches();
    return;
  }

  // Polling fallback
  const uint32_t now = millis();
  if (now - this->last_poll_ms_ >= this->polling_fallback_ms_) {
    this->last_poll_ms_ = now;
    this->update_touches();
  }
}

void SPD2010Touch::update_touches() {
  TouchFrame frame{};
  this->tp_read_data_(&frame);

  static uint32_t last = 0;
  uint32_t now = millis();
  if (now - last > 500) {
    last = now;
    ESP_LOGD(TAG, "Reading touch...");
    ESP_LOGD(TAG, "touch_num=%u", frame.touch_num);
  }

  for (uint8_t i = 0; i < frame.touch_num && i < 5; i++) {
    this->add_raw_touch_position_(frame.rpt[i].id, frame.rpt[i].x, frame.rpt[i].y, frame.rpt[i].weight);
  }
  this->send_touches_();
}

// ---------- I2C helpers (16-bit register addressing) ----------
bool SPD2010Touch::read16_(uint16_t reg, uint8_t *data, size_t len) {
  uint8_t regbuf[2] = { (uint8_t)(reg >> 8), (uint8_t)(reg & 0xFF) };

  // Combined transaction: write reg pointer then read (repeated start)
  return this->write_read(regbuf, 2, data, len);
}


bool SPD2010Touch::write16_(uint16_t reg, const uint8_t *data, size_t len) {
  // Build buffer: [reg_lo reg_hi payload...]
  uint8_t buf[2 + 8];
  if (len > 8) return false;

  buf[0] = (uint8_t)(reg >> 8);
  buf[1] = (uint8_t)(reg & 0xFF);

  for (size_t i = 0; i < len; i++) {
    buf[2 + i] = data[i];
  }

  return this->write(buf, 2 + len);
}


// ---------- Commands (ported from your functions) ----------
void SPD2010Touch::write_tp_point_mode_cmd_() {
  const uint8_t payload[2] = {0x00, 0x00};
  this->write16_(REG_POINT_MODE, payload, 2);
}

void SPD2010Touch::write_tp_start_cmd_() {
  const uint8_t payload[2] = {0x00, 0x00};
  this->write16_(REG_TOUCH_START, payload, 2);
}

void SPD2010Touch::write_tp_cpu_start_cmd_() {
  const uint8_t payload[2] = {0x01, 0x00};
  this->write16_(REG_CPU_START, payload, 2);
}

void SPD2010Touch::write_tp_clear_int_cmd_() {
  const uint8_t payload[2] = {0x01, 0x00};
  this->write16_(REG_CLEAR_INT, payload, 2);
}

void SPD2010Touch::read_tp_status_length_(TpStatus *st) {
  uint8_t d[4]{0};

  bool ok = this->read16_(REG_STATUS_LEN, d, 4);
  if (!ok) {
    ESP_LOGD(TAG, "STATUS read failed raw=%02X %02X %02X %02X", d[0], d[1], d[2], d[3]);
    return;
  }
  ESP_LOGD(TAG, "STATUS read ok raw=%02X %02X %02X %02X", d[0], d[1], d[2], d[3]);

  st->low.pt_exist = (d[0] & 0x01);
  st->low.gesture  = (d[0] & 0x02);
  st->low.aux      = (d[0] & 0x08);

  st->high.tic_busy    = (d[1] & 0x80) >> 7;
  st->high.tic_in_bios = (d[1] & 0x40) >> 6;
  st->high.tic_in_cpu  = (d[1] & 0x20) >> 5;
  st->high.tint_low    = (d[1] & 0x10) >> 4;
  st->high.cpu_run     = (d[1] & 0x08) >> 3;

  st->read_len = (static_cast<uint16_t>(d[3]) << 8) | d[2];
}

void SPD2010Touch::read_tp_hdp_(const TpStatus &st, TouchFrame *frame) {
  // your code: header 4 bytes + (n * 6)
  if (st.read_len < 4 || st.read_len > (4 + 10 * 6)) {
    frame->touch_num = 0;
    return;
  }

  uint8_t buf[4 + 10 * 6]{0};
  this->read16_(REG_HDP_READ, buf, st.read_len);

  const uint8_t check_id = buf[4];

  if (check_id <= 0x0A && st.low.pt_exist) {
    const uint8_t touch_num = (st.read_len - 4) / 6;
    frame->touch_num = touch_num > 5 ? 5 : touch_num;

    for (uint8_t i = 0; i < frame->touch_num; i++) {
      const uint8_t off = i * 6;
      frame->rpt[i].id = buf[4 + off];
      frame->rpt[i].x = static_cast<uint16_t>(((buf[7 + off] & 0xF0) << 4) | buf[5 + off]);
      frame->rpt[i].y = static_cast<uint16_t>(((buf[7 + off] & 0x0F) << 8) | buf[6 + off]);
      frame->rpt[i].weight = buf[8 + off];
    }
  } else {
    frame->touch_num = 0;
  }
}

void SPD2010Touch::read_tp_hdp_status_(TpHdpStatus *hs) {
  uint8_t d[8]{0};
  this->read16_(REG_HDP_STATUS, d, 8);
  hs->status = d[5];
  hs->next_packet_len = static_cast<uint16_t>(d[2]) | (static_cast<uint16_t>(d[3]) << 8);
}

void SPD2010Touch::read_hdp_remain_data_(const TpHdpStatus &hs) {
  if (hs.next_packet_len == 0 || hs.next_packet_len > 32) return;
  uint8_t dummy[32]{0};
  this->read16_(REG_HDP_READ, dummy, hs.next_packet_len);
}

void SPD2010Touch::tp_read_data_(TouchFrame *frame) {
  TpStatus st{};
  TpHdpStatus hs{};

  this->read_tp_status_length_(&st);
  
  static uint32_t last_st = 0;
  uint32_t now = millis();
  if (now - last_st > 500) {
    last_st = now;
    ESP_LOGD(TAG, "pt_exist=%u gesture=%u aux=%u read_len=%u cpu_run=%u in_cpu=%u in_bios=%u",
             st.low.pt_exist, st.low.gesture, st.low.aux, st.read_len,
             st.high.cpu_run, st.high.tic_in_cpu, st.high.tic_in_bios);
  }

  if (st.high.tic_in_bios) {
    this->write_tp_clear_int_cmd_();
    this->write_tp_cpu_start_cmd_();
    frame->touch_num = 0;
    return;
  }

  if (st.high.tic_in_cpu) {
    this->write_tp_point_mode_cmd_();
    this->write_tp_start_cmd_();
    this->write_tp_clear_int_cmd_();
    frame->touch_num = 0;
    return;
  }

  if (st.high.cpu_run && st.read_len == 0) {
    this->write_tp_clear_int_cmd_();
    frame->touch_num = 0;
    return;
  }

  if (st.low.pt_exist || st.low.gesture) {
    this->read_tp_hdp_(st, frame);

  hdp_done_check:
    this->read_tp_hdp_status_(&hs);
    if (hs.status == 0x82) {
      this->write_tp_clear_int_cmd_();
    } else if (hs.status == 0x00) {
      this->read_hdp_remain_data_(hs);
      goto hdp_done_check;
    }
    return;
  }

  if (st.high.cpu_run && st.low.aux) {
    this->write_tp_clear_int_cmd_();
  }

  frame->touch_num = 0;
}

}  // namespace spd2010_touch
}  // namespace esphome











