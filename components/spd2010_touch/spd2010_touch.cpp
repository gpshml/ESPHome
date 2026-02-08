#include "spd2010_touch.h"
#include "inttypes.h"
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
static constexpr uint16_t REG_FW_INFO     = 0x2600; // read 18 bytes

void SPD2010Touch::notify_display_ready() {
  this->display_ready_ = true;

  // Force a clean init cycle
  this->initialised_ = false;
  this->init_attempts_ = 0;

  // Very important: touch reset AFTER display init (0x29 + delay)
  this->pulse_shared_reset_();

  // Give the SPD2010 a moment to come up after reset
  this->next_init_try_ms_ = millis() + 150;

  ESP_LOGI(TAG, "Display marked ready; touch reset pulsed; init scheduled (I2C 0x%02X).", this->address_);
}

void SPD2010Touch::setup() {
  ESP_LOGCONFIG(TAG, "Setting up SPD2010 touch... Address 0x%02X", this->address_);
  this->boot_ms_ = millis();

  // Optional: configure IRQ
  if (this->irq_pin_ != nullptr) {
    this->irq_pin_->setup();
    this->irq_pin_->attach_interrupt(SPD2010Touch::gpio_isr_, this, gpio::INTERRUPT_FALLING_EDGE);
  }

  // Don’t hard-fail if not ready yet; we’ll retry in loop()
  this->initialised_ = false;
  this->next_probe_ms_ = 0;
  ESP_LOGD(TAG, "Touch will probe until 0x53 ACKs...");
}

void SPD2010Touch::dump_config() {
  ESP_LOGCONFIG(TAG, "SPD2010 Touch:");
  LOG_I2C_DEVICE(this);
  LOG_PIN("  Interrupt Pin: ", this->irq_pin_);
  ESP_LOGCONFIG(TAG, "  Polling fallback: %ums", this->polling_fallback_ms_);
  ESP_LOGCONFIG(TAG, "  Display-ready gating: %s", this->display_ready_ ? "YES" : "NO");
  ESP_LOGCONFIG(TAG, "  Initialised: %s", this->initialised_ ? "YES" : "NO");
}

void SPD2010Touch::loop() {
  const uint32_t now = millis();

  // Phase 0: don’t do anything until the display says it’s ready
  if (!this->display_ready_) {
    return;
  }

  // Phase 1: deferred init (includes reset pulse + ACK gating)
  if (this->display_ready_ && !this->initialised_ && now >= this->next_init_try_ms_) {
  
    // pulse reset (via expander) every few attempts
    this->init_attempts_++;
    if ((this->init_attempts_ % 5) == 0) {
      this->pulse_shared_reset_();
    }
  
    if (!this->probe_ack_()) {
      ESP_LOGW(TAG, "SPD2010 not ACKing 0x%02X (attempt %u).", this->address_, this->init_attempts_);
      this->log_i2c_probe_scan_();
      this->next_init_try_ms_ = now + 500;
      return;
    }

    delay(50);

    // Now try your real init sequence (writes/register setup)
    this->try_init_();

    // If init didn’t stick, back off and retry
    if (!this->initialised_) {
      this->next_init_try_ms_ = now + 500;
      return;
    }
  }

  // Phase 2: touch readout (only after init)
  if (!this->initialised_) {
    return;
  }

  if (this->irq_fired_ || (now - this->last_poll_ms_ > this->polling_fallback_ms_)) {
    this->irq_fired_ = false;
    this->last_poll_ms_ = now;
    this->update_touches();
  }
}

void SPD2010Touch::try_init_() {
  uint8_t payload[2];

  // CPU_START
  payload[0] = 0x01; payload[1] = 0x00;
  if (!this->write16_(REG_CPU_START, payload, 2)) {
    ESP_LOGW(TAG, "Init failed: CPU_START write NACK");
    this->initialised_ = false;
    return;
  }
  delay(20);

  // POINT_MODE
  payload[0] = 0x00; payload[1] = 0x00;
  if (!this->write16_(REG_POINT_MODE, payload, 2)) {
    ESP_LOGW(TAG, "Init failed: POINT_MODE write NACK");
    this->initialised_ = false;
    return;
  }
  delay(20);

  // TOUCH_START
  payload[0] = 0x00; payload[1] = 0x00;
  if (!this->write16_(REG_TOUCH_START, payload, 2)) {
    ESP_LOGW(TAG, "Init failed: TOUCH_START write NACK");
    this->initialised_ = false;
    return;
  }
  delay(20);

  // CLEAR_INT
  payload[0] = 0x01; payload[1] = 0x00;
  if (!this->write16_(REG_CLEAR_INT, payload, 2)) {
    ESP_LOGW(TAG, "Init failed: CLEAR_INT write NACK");
    this->initialised_ = false;
    return;
  }
  delay(20);

  // Probe status register 0x2000
  uint8_t status_buf[4] = {0};
  if (!this->read16_(REG_STATUS_LEN, status_buf, 4)) {
    ESP_LOGW(TAG, "Init failed: STATUS read (0x2000) failed");
    this->initialised_ = false;
    return;
  }

  ESP_LOGI(TAG, "STATUS: %02X %02X %02X %02X",
           status_buf[0], status_buf[1], status_buf[2], status_buf[3]);

  // Read-only FW probe (0x2600, 18 bytes)
  uint16_t dver = 0;
  uint32_t pid = 0, name_hi = 0, name_lo = 0;
  (void) this->read_fw_info_(&dver, &pid, &name_hi, &name_lo);

  this->initialised_ = true;
  ESP_LOGI(TAG, "SPD2010 touch initialised and active.");

}
void SPD2010Touch::update_touches() {
  TouchFrame frame{};
  this->tp_read_data_(&frame);

  if (frame.touch_num > 0) {
    ESP_LOGD(TAG, "touch_num=%u", frame.touch_num);
  }

  for (uint8_t i = 0; i < frame.touch_num && i < 5; i++) {
    this->add_raw_touch_position_(frame.rpt[i].id, frame.rpt[i].x, frame.rpt[i].y, frame.rpt[i].weight);
  }
  this->send_touches_();
}

// ---------- I2C helpers (16-bit register addressing, ESPHome-native) ----------
bool SPD2010Touch::read16_(uint16_t reg, uint8_t *data, size_t len) {
  const auto err = this->read_register16(reg, data, len);
  if (err != esphome::i2c::ERROR_OK) {
    ESP_LOGV(TAG, "read16: reg=0x%04X len=%u err=%d", reg, (unsigned) len, (int) err);
    return false;
  }
  return true;
}

bool SPD2010Touch::write16_(uint16_t reg, const uint8_t *data, size_t len) {
  const auto err = this->write_register16(reg, data, len);
  if (err != esphome::i2c::ERROR_OK) {
    ESP_LOGV(TAG, "write16: reg=0x%04X len=%u err=%d", reg, (unsigned) len, (int) err);
    return false;
  }
  return true;
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

void SPD2010Touch::pulse_shared_reset_() {
  if (this->reset_expander_ == nullptr || this->reset_expander_pin_ > 7) return;

  // Active-low reset pulse
  this->reset_expander_->digital_write(this->reset_expander_pin_, false);
  delay(20);
  this->reset_expander_->digital_write(this->reset_expander_pin_, true);
  delay(80);
}

bool SPD2010Touch::probe_ack_() {
  // 0-byte write = address probe
  esphome::i2c::ErrorCode err = this->write(nullptr, 0);
  return err == esphome::i2c::ERROR_OK;
}

void SPD2010Touch::log_i2c_probe_scan_() {
  static uint32_t last_scan = 0;
  const uint32_t now = millis();
  if (now - last_scan < 5000) return;
  last_scan = now;

  const uint8_t original = this->address_;
  ESP_LOGW(TAG, "SPD2010 not ACKing 0x%02X. Probing 0x08–0x77 for late responders...", original);

  for (uint8_t a = 0x08; a <= 0x77; a++) {
    this->set_i2c_address(a);          // <-- correct method name
    if (this->probe_ack_()) {
      ESP_LOGW(TAG, "I2C responder detected at 0x%02X", a);
    }
  }

  this->set_i2c_address(original);      // <-- restore
}

bool SPD2010Touch::read_fw_info_(uint16_t *dver_out, uint32_t *pid_out, uint32_t *name_hi_out, uint32_t *name_lo_out) {
  uint8_t b[18] = {0};

  if (!this->read16_(REG_FW_INFO, b, sizeof(b))) {
    ESP_LOGW(TAG, "FW probe failed: read 0x%04X len=18", REG_FW_INFO);
    return false;
  }

  // Mirrors the Arduino parsing (Touch_SPD2010.cpp)
  const uint16_t dver = (static_cast<uint16_t>(b[5]) << 8) | b[4];
  const uint32_t pid  = (static_cast<uint32_t>(b[9])  << 24) |
                        (static_cast<uint32_t>(b[8])  << 16) |
                        (static_cast<uint32_t>(b[7])  << 8)  |
                        (static_cast<uint32_t>(b[6]));
  const uint32_t name_lo = (static_cast<uint32_t>(b[13]) << 24) |
                           (static_cast<uint32_t>(b[12]) << 16) |
                           (static_cast<uint32_t>(b[11]) << 8)  |
                           (static_cast<uint32_t>(b[10]));
  const uint32_t name_hi = (static_cast<uint32_t>(b[17]) << 24) |
                           (static_cast<uint32_t>(b[16]) << 16) |
                           (static_cast<uint32_t>(b[15]) << 8)  |
                           (static_cast<uint32_t>(b[14]));

  if (dver_out) *dver_out = dver;
  if (pid_out) *pid_out = pid;
  if (name_hi_out) *name_hi_out = name_hi;
  if (name_lo_out) *name_lo_out = name_lo;

  ESP_LOGI(TAG, "FW: DVer=%u PID=%" PRIu32 " NameHi=0x%08" PRIX32 " NameLo=0x%08" PRIX32,
           (unsigned) dver, pid, name_hi, name_lo);

  return true;
}

void SPD2010Touch::read_tp_status_length_(TpStatus *st) {
  uint8_t d[4]{0};

  bool ok = this->read16_(REG_STATUS_LEN, d, 4);
  if (!ok) {
    // If it's all zeros, treat as dead/no-data. Otherwise parse anyway.
    if (d[0] == 0x00 && d[1] == 0x00 && d[2] == 0x00 && d[3] == 0x00) {
      st->low.pt_exist = 0;
      st->low.gesture = 0;
      st->low.aux = 0;
      st->high.tic_in_bios = 0;
      st->high.tic_in_cpu = 0;
      st->high.cpu_run = 0;
      st->read_len = 0;
      return;
    }
  }

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

  if (st.high.tic_in_bios) {
    this->write_tp_clear_int_cmd_();
    this->write_tp_cpu_start_cmd_();
    this->write_tp_point_mode_cmd_();
    this->write_tp_start_cmd_();
    this->write_tp_clear_int_cmd_();
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

    uint8_t retries = 0;
  hdp_done_check:
    this->read_tp_hdp_status_(&hs);
    if (hs.status == 0x82) {
      this->write_tp_clear_int_cmd_();
    } else if (hs.status == 0x00) {
      this->read_hdp_remain_data_(hs);
      if (++retries > 5) {
        ESP_LOGW(TAG, "HDP loop exceeded retries");
        this->write_tp_clear_int_cmd_();
        frame->touch_num = 0;
        return;
      }
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





