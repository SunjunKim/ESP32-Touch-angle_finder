// Using library Driver Bus Library Based on Arduino v1.0.1
// Using library GFX Library for Arduino v1.6.5
// Using library Mylibrary (legacy)
// Using library SensorLib v0.2.1
// Using library lvgl v8.4.0
// Using library Networking v3.3.7
// Using library SPI v3.3.7
// Using library WiFi v3.3.7
// Using library Wire v3.3.7

#include <stdint.h>

// Forward declaration to survive Arduino auto-prototypes.
enum class AppMode : uint8_t;

#include <Arduino.h>
#include <lvgl.h>
#include <Wire.h>
#include <esp_timer.h>
#include <esp_sleep.h>
#include <driver/gpio.h>
#include <freertos/semphr.h>
#include <memory>

#include "Arduino_GFX_Library.h"
#include "Arduino_DriveBus_Library.h"
#include "pin_config.h"
#include "lv_conf.h"

#include "SensorQMI8658.hpp"
#include "HWCDC.h"
#include "WiFi.h"

#include "src/pages/abs_page.h"
#include "src/pages/rel_page.h"
#include "src/pages/page_types.h"

LV_FONT_DECLARE(lv_font_roboto_mono_48);

HWCDC USBSerial;

const int beePin = 42;     // Define GPIO42 for buzzer
static constexpr int kSysEnPin = 41; // Schematic: SYS_EN -> GPIO41, set as OUTPUT
static constexpr int kSysOutPin = 40; // Schematic: SYS_OUT -> GPIO40 -> PWR btn, set as INPUT

static constexpr int kModePin = 0; // Schematic: GPIO0 = boot / used as Mode Button
static constexpr int kImuIntPin = 38; // QMI8658 INT1 -> GPIO38

// ---- Buzzer helpers ----
static bool g_beepActive = false;
static uint32_t g_beepUntilMs = 0;

static inline void start_beep(uint16_t hz, uint32_t durationMs)
{
  tone(beePin, hz);
  g_beepActive = true;
  g_beepUntilMs = millis() + durationMs;
}

static inline void beep_poll()
{
  if (!g_beepActive) return;
  if ((int32_t)(millis() - g_beepUntilMs) < 0) return;
  g_beepActive = false;
  noTone(beePin);
}



static inline void play_welcome_sound()
{
  // Short, bright startup arpeggio (~240ms total).
  // Using tone()/noTone() for consistency with the rest of this sketch.
  static constexpr uint16_t kNotesHz[] = { 1047, 1319, 1568, 2093 }; // C6 E6 G6 C7
  for (uint16_t hz : kNotesHz) {
    tone(beePin, hz);
    delay(45);
    noTone(beePin);
    delay(15);
  }
}

static TaskHandle_t g_imuTaskHandle = nullptr;

// ---- LVGL tick ----
// Tick period for LVGL internal timing (50 Hz).
#define EXAMPLE_LVGL_TICK_PERIOD_MS 20

// ---- App cadence (battery/perf) ----
// Target ~30 Hz “frame rate” for LVGL work.
static constexpr uint32_t kUiFramePeriodMs = 33; // ~30.3 Hz

// ---- Backlight PWM + inactivity dimming ----
// Dims after 10s of no button/touch and no significant motion.
static constexpr uint32_t kInactivityDimAfterMs = 5'000;
static constexpr uint32_t kMotionActivityHoldMs = 2'000; // keep "active" briefly after motion

// ---- Power-off ("sleep") ----
// Requirements:
// - 15s of inactivity -> enter ESP sleep (wake via SYS_OUT / PWR button)
// - Long-press PWR (SYS_OUT LOW while pressed) -> enter ESP sleep
static constexpr uint32_t kInactivitySleepAfterMs = 15'000;
static constexpr uint32_t kPwrLongPressMs = 2'000;
static constexpr uint32_t kPwrDebounceMs = 50;
static constexpr uint32_t kPwrIgnoreAfterBootMs = 3'000;
static constexpr uint32_t kPwrReleaseWaitTimeoutMs = 5'000;
static constexpr uint32_t kPwrIgnoreAfterWakeMs = 1'500;

static constexpr uint8_t kBacklightFull = 255; // 0..255
static constexpr uint8_t kBacklightDim  = 25;  // tweak to taste (lower = darker, less power)

static constexpr uint32_t kBacklightPwmHz = 5000;
static constexpr uint8_t  kBacklightPwmResBits = 8; // duty 0..255
static constexpr int      kBacklightPwmChannel = 0; // legacy cores only (kept for reference)

static uint8_t g_backlightDuty = kBacklightFull;
static uint32_t g_lastUserActivityMs = 0;
static uint32_t g_lastMotionActivityMs = 0;

static inline void backlight_pwm_init()
{
  // ESP32 Arduino core 3.x (your ESP32 3.3.7): LEDC pin-based API.
  // Note: legacy ledcSetup/ledcAttachPin are not available in core 3.x.
  ledcAttach(LCD_BL, kBacklightPwmHz, kBacklightPwmResBits);
}

static inline void backlight_set(uint8_t duty)
{
  g_backlightDuty = duty;
  ledcWrite(LCD_BL, duty);
}

static inline void mark_user_activity()
{
  g_lastUserActivityMs = millis();
}

static inline void mark_motion_activity()
{
  g_lastMotionActivityMs = millis();
}

static void backlight_idle_manager()
{
  const uint32_t now = millis();
  const uint32_t lastAny = (g_lastMotionActivityMs > g_lastUserActivityMs) ? g_lastMotionActivityMs : g_lastUserActivityMs;
  const bool motionRecentlyActive = (now - g_lastMotionActivityMs) < kMotionActivityHoldMs;
  const uint32_t idleForMs = now - lastAny;
  const bool shouldDim = !motionRecentlyActive && idleForMs >= kInactivityDimAfterMs;

  const uint8_t target = shouldDim ? kBacklightDim : kBacklightFull;
  if (target != g_backlightDuty) {
    backlight_set(target);
  }
}

static uint32_t g_bootMs = 0;
static uint32_t g_wakeMs = 0;
static bool g_powerOffInitiated = false;

// ---- Long-press detection state (resettable) ----
static int g_pwrLastLevel = HIGH;
static uint32_t g_pwrLastLevelChangeMs = 0;
static bool g_pwrPressedStable = false;
static uint32_t g_pwrPressStartMs = 0;

static inline void reset_power_button_state()
{
  g_pwrLastLevel = digitalRead(kSysOutPin);
  g_pwrLastLevelChangeMs = millis();
  g_pwrPressedStable = false;
  g_pwrPressStartMs = 0;
}

static void enter_poweroff_sleep(const char *reason, const bool wait_for_pwr_release)
{
  if (g_powerOffInitiated) return;
  g_powerOffInitiated = true;

  // Best-effort logs; USBSerial/screen power may stop once we enter sleep.
  if (reason) {
    USBSerial.print("Power off: ");
    USBSerial.println(reason);
    USBSerial.flush();
  }

  // Audible confirmation before cutting power: Descending "do-sol-mi-do" (C-G-E-C) tone
  tone(beePin, 523); // C5 (do)
  delay(80);
  tone(beePin, 392); // G4 (sol)
  delay(80);
  tone(beePin, 330); // E4 (mi)
  delay(80);
  tone(beePin, 261); // C4 (do, lower octave)
  delay(100);
  noTone(beePin);

  // Make the backlight transition look nicer before cutting power.
  backlight_set(kBacklightDim);
  delay(30);

  // Try the original "hard power-off" path first.
  // - On battery: SYS_EN LOW should physically cut VBAT (no code after this runs).
  // - On USB: SYS_EN LOW often doesn't fully power down, so we fall back to ESP sleep.
  digitalWrite(kSysEnPin, LOW);
  delay(50);

  // Ensure a clean sleep entry for long-press: SYS_OUT is LOW while pressed.
  // If we enter sleep while it is already LOW and wake is configured on LOW,
  // the CPU may wake immediately. So wait for release before sleeping.
  if (wait_for_pwr_release) {
    const uint32_t startMs = millis();
    while (digitalRead(kSysOutPin) == LOW && (millis() - startMs) < kPwrReleaseWaitTimeoutMs) {
      delay(10);
    }
  }

  // IMPORTANT:
  // - SYS_OUT is on GPIO40.
  // - On ESP32-S3, deep-sleep EXT0/EXT1 wake requires RTC IO pins; GPIO40 is NOT RTC IO.
  // Therefore, use LIGHT SLEEP with GPIO wakeup for the USB fallback path.

  // Configure wake on SYS_OUT (PWR button) asserting LOW.
  // (This is level-based; a short press is enough to wake.)
  gpio_wakeup_enable((gpio_num_t)kSysOutPin, GPIO_INTR_LOW_LEVEL);
  esp_sleep_enable_gpio_wakeup();

  // Drop backlight to minimize draw before the CPU sleeps.
  backlight_set(0);
  delay(10);

  // Enter light sleep; execution resumes here after wake.
  esp_light_sleep_start();

  // After wake: re-latch SYS_EN high (in case SYS_EN LOW partially affected rails on USB).
  digitalWrite(kSysEnPin, HIGH);
  backlight_set(kBacklightFull);
  mark_user_activity();
  g_wakeMs = millis();
  reset_power_button_state();
  g_powerOffInitiated = false;
}

static void power_sleep_manager()
{
  const uint32_t now = millis();
  if (now - g_bootMs < kPwrIgnoreAfterBootMs) return; // don't sleep during boot/setup
  if (g_wakeMs != 0 && (now - g_wakeMs) < kPwrIgnoreAfterWakeMs) return; // don't sleep right after wake

  // ---- Long-press detection on SYS_OUT (PWR button) ----
  // SYS_OUT is LOW while the PWR button is pressed.
  const int level = digitalRead(kSysOutPin);
  if (level != g_pwrLastLevel) {
    g_pwrLastLevel = level;
    g_pwrLastLevelChangeMs = now;
  }

  if ((now - g_pwrLastLevelChangeMs) >= kPwrDebounceMs) {
    const bool pressed = (level == LOW);
    if (pressed) {
      if (!g_pwrPressedStable) {
        g_pwrPressedStable = true;
        g_pwrPressStartMs = now;
      } else {
        const uint32_t heldMs = now - g_pwrPressStartMs;
        if (heldMs >= kPwrLongPressMs) {
          enter_poweroff_sleep("PWR long press", /*wait_for_pwr_release=*/true);
        }
      }
    } else {
      g_pwrPressedStable = false;
    }
  }

  // ---- Inactivity sleep ----
  const uint32_t lastAny = (g_lastMotionActivityMs > g_lastUserActivityMs) ? g_lastMotionActivityMs : g_lastUserActivityMs;
  if (lastAny != 0 && (now - lastAny) >= kInactivitySleepAfterMs) {
    enter_poweroff_sleep("inactivity", /*wait_for_pwr_release=*/false);
  }
}

static lv_disp_draw_buf_t draw_buf;
static lv_color_t buf[LCD_WIDTH * LCD_HEIGHT / 10];

Arduino_DataBus *bus = new Arduino_ESP32SPI(LCD_DC, LCD_CS, LCD_SCK, LCD_MOSI);
Arduino_GFX *gfx = new Arduino_ST7789(
  bus,
  LCD_RST /* RST */,
  0 /* rotation */,
  true /* IPS */,
  LCD_WIDTH,
  LCD_HEIGHT,
  0,
  20,
  0,
  0
);

#if LV_USE_LOG != 0
static void my_print(const char *buf)
{
  USBSerial.printf(buf);
  USBSerial.flush();
}
#endif

static void my_disp_flush(lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *color_p)
{
  const uint32_t w = (area->x2 - area->x1 + 1);
  const uint32_t h = (area->y2 - area->y1 + 1);

#if (LV_COLOR_16_SWAP != 0)
  gfx->draw16bitBeRGBBitmap(area->x1, area->y1, (uint16_t *)&color_p->full, w, h);
#else
  gfx->draw16bitRGBBitmap(area->x1, area->y1, (uint16_t *)&color_p->full, w, h);
#endif

  lv_disp_flush_ready(disp);
}

static void lvgl_tick_cb(void * /*arg*/)
{
  lv_tick_inc(EXAMPLE_LVGL_TICK_PERIOD_MS);
}

// ---- App: modes ----
enum class AppMode : uint8_t {
  AbsAngle = 0,
  RelAngle = 1,
};

static AppMode g_mode = AppMode::RelAngle;
static constexpr uint8_t kAppModeCount = 2;

static inline AppMode next_mode(AppMode m)
{
  return (AppMode)(((uint8_t)m + 1u) % kAppModeCount);
}

// ---- Mode button (short-press to switch ABS/REL) ----
static constexpr uint32_t kModeDebounceMs = 30;
static constexpr uint32_t kModeShortPressMaxMs = 500;
static constexpr uint16_t kModeSwitchBeepHz = 440;
static constexpr uint32_t kModeSwitchBeepMs = 60;

struct ModeButtonState {
  uint8_t lastReading = HIGH;
  uint8_t stable = HIGH;
  uint32_t lastChangeMs = 0;
  bool wasPressed = false;
  uint32_t pressStartMs = 0;
};

static ModeButtonState g_modeButton;

static void mode_button_poll()
{
  const uint32_t now = millis();
  const uint8_t reading = (uint8_t)digitalRead(kModePin);

  if (reading != g_modeButton.lastReading) {
    g_modeButton.lastReading = reading;
    g_modeButton.lastChangeMs = now;
  }

  const bool debounced = (now - g_modeButton.lastChangeMs) >= kModeDebounceMs;
  const bool changed = (g_modeButton.stable != g_modeButton.lastReading);
  if (!debounced || !changed) return;

  g_modeButton.stable = g_modeButton.lastReading;
  if (g_modeButton.stable == LOW) {
    g_modeButton.wasPressed = true;
    g_modeButton.pressStartMs = now;
    mark_user_activity();
    return;
  }

  if (!g_modeButton.wasPressed) return;
  g_modeButton.wasPressed = false;

  const uint32_t heldMs = now - g_modeButton.pressStartMs;
  if (heldMs > 0 && heldMs <= kModeShortPressMaxMs) {
    g_mode = next_mode(g_mode);
    mark_user_activity();
    start_beep(kModeSwitchBeepHz, kModeSwitchBeepMs);
  }
}

// ---- Touch (CST816T) ----
static std::shared_ptr<Arduino_IIC_DriveBus> IIC_Bus =
  std::make_shared<Arduino_HWIIC>(IIC_SDA, IIC_SCL, &Wire);

static void Arduino_IIC_Touch_Interrupt(void);

static std::unique_ptr<Arduino_IIC> CST816T(
  new Arduino_CST816x(IIC_Bus, CST816T_DEVICE_ADDRESS, TP_RST, TP_INT, Arduino_IIC_Touch_Interrupt)
);

static void Arduino_IIC_Touch_Interrupt(void)
{
  CST816T->IIC_Interrupt_Flag = true;
}

static void my_touchpad_read(lv_indev_drv_t * /*indev_driver*/, lv_indev_data_t *data)
{
  if (CST816T->IIC_Interrupt_Flag == true) {
    CST816T->IIC_Interrupt_Flag = false;
    data->state = LV_INDEV_STATE_PR;
    mark_user_activity();
    const int32_t touchX = CST816T->IIC_Read_Device_Value(
      CST816T->Arduino_IIC_Touch::Value_Information::TOUCH_COORDINATE_X
    );
    const int32_t touchY = CST816T->IIC_Read_Device_Value(
      CST816T->Arduino_IIC_Touch::Value_Information::TOUCH_COORDINATE_Y
    );
    if (touchX >= 0 && touchY >= 0) {
      data->point.x = touchX;
      data->point.y = touchY;
    }
  } else {
    data->state = LV_INDEV_STATE_REL;
  }
}

// ---- Battery (from 07_LVGL_Measuring_voltage) ----
static constexpr int kVoltageDividerPin = 1; // GPIO1
static constexpr float kVRef = 3.3f;
static constexpr float kR1 = 200000.0f;
static constexpr float kR2 = 100000.0f;

static float g_battVoltage = NAN;
static uint8_t g_battPct = 0;

// ---- IMU / angle state ----
static SensorQMI8658 qmi;
static SemaphoreHandle_t g_imuIntSem = nullptr;

static void IRAM_ATTR imu_int_isr()
{
  BaseType_t higherPrioTaskWoken = pdFALSE;
  if (g_imuIntSem != nullptr) {
    xSemaphoreGiveFromISR(g_imuIntSem, &higherPrioTaskWoken);
  }
  if (higherPrioTaskWoken == pdTRUE) {
    portYIELD_FROM_ISR();
  }
}

struct ImuState {
  float ax_g = 0.0f;
  float ay_g = 0.0f;
  float az_g = 0.0f;

  float pitch_deg = 0.0f;
  float roll_deg = 0.0f;

  // REL mode: reference plane normal captured at "zero" time
  float ref_ax_g = 0.0f;
  float ref_ay_g = 0.0f;
  float ref_az_g = 0.0f;
  bool ref_valid = false;

  // REL mode: angle between current normal and reference normal (degrees)
  float rel_angle_deg = 0.0f;

  bool inited = false;
};

static ImuState g_imu;
static portMUX_TYPE g_imuMux = portMUX_INITIALIZER_UNLOCKED;

static inline float clampf(float v, float lo, float hi)
{
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

static inline float rad2deg(float r) { return r * 57.29577951308232f; }

static inline float round_to_step(float v, float step)
{
  if (!(step > 0.0f) || !isfinite(v)) return v;
  return roundf(v / step) * step;
}

static inline float safe_norm3(float x, float y, float z)
{
  const float n2 = x * x + y * y + z * z;
  if (n2 <= 1e-12f) return 0.0f;
  return sqrtf(n2);
}

static inline void normalize3(float &x, float &y, float &z)
{
  const float n = safe_norm3(x, y, z);
  if (n <= 0.0f) return;
  x /= n;
  y /= n;
  z /= n;
}

static void imu_set_reference_normal_now()
{
  portENTER_CRITICAL(&g_imuMux);
  g_imu.ref_ax_g = g_imu.ax_g;
  g_imu.ref_ay_g = g_imu.ay_g;
  g_imu.ref_az_g = g_imu.az_g;
  g_imu.ref_valid = (safe_norm3(g_imu.ref_ax_g, g_imu.ref_ay_g, g_imu.ref_az_g) > 0.0f);
  portEXIT_CRITICAL(&g_imuMux);
}

static void imu_task(void * /*param*/)
{
  // Bring up QMI8658 in a simple raw-read configuration (acc + gyro).
  if (!qmi.begin(Wire, QMI8658_L_SLAVE_ADDRESS, IIC_SDA, IIC_SCL)) {
    portENTER_CRITICAL(&g_imuMux);
    g_imu.inited = false;
    portEXIT_CRITICAL(&g_imuMux);
    vTaskDelete(NULL);
  }

  // "Stable" choice: modest ODR + LPF enabled.
  qmi.configAccelerometer(
    SensorQMI8658::ACC_RANGE_2G,
    SensorQMI8658::ACC_ODR_250Hz,
    SensorQMI8658::LPF_MODE_0,
    true,   // LPF enable
    false   // self-test disable
  );
  qmi.enableAccelerometer();

  qmi.configGyroscope(
    SensorQMI8658::GYR_RANGE_128DPS,
    SensorQMI8658::GYR_ODR_224_2Hz,
    SensorQMI8658::LPF_MODE_0,
    true,
    false
  );
  qmi.enableGyroscope();

  qmi.enableLockingMechanism();
  qmi.enableINT(SensorQMI8658::IntPin1);

  // INT-driven IMU reads: wait for data-ready interrupt.
  // Keep a timeout as a resilience fallback.
  const TickType_t intWait = pdMS_TO_TICKS(100);
  float prev_ax = 0.0f, prev_ay = 0.0f, prev_az = 0.0f;
  bool prev_valid = false;
  static constexpr float kMotionDeltaG = 0.08f;

  portENTER_CRITICAL(&g_imuMux);
  g_imu.inited = true;
  portEXIT_CRITICAL(&g_imuMux);

  for (;;) {
    if (g_imuIntSem != nullptr) {
      (void)xSemaphoreTake(g_imuIntSem, intWait);
    } else {
      vTaskDelay(pdMS_TO_TICKS(20));
    }

    if (!qmi.getDataReady()) {
      continue;
    }

    float ax, ay, az, gx, gy, gz;
    if (!qmi.getAccelerometer(ax, ay, az)) {
      continue;
    }
    if (!qmi.getGyroscope(gx, gy, gz)) {
      continue;
    }

    // Raw accelerometer vector is used directly.
    const float pitch = rad2deg(atan2f(ax, sqrtf(ay * ay + az * az)));
    const float roll  = rad2deg(atan2f(ay, sqrtf(ax * ax + az * az)));

    if (!prev_valid) {
      prev_ax = ax;
      prev_ay = ay;
      prev_az = az;
      prev_valid = true;
      mark_motion_activity();
    } else {
      const float delta = safe_norm3(ax - prev_ax, ay - prev_ay, az - prev_az);
      if (delta >= kMotionDeltaG) {
        mark_motion_activity();
      }
      prev_ax = ax;
      prev_ay = ay;
      prev_az = az;
    }

    portENTER_CRITICAL(&g_imuMux);
    // Expose raw accelerometer readings from the chip.
    g_imu.ax_g = ax;
    g_imu.ay_g = ay;
    g_imu.az_g = az;

    g_imu.pitch_deg = pitch;
    g_imu.roll_deg = roll;

    // REL mode: angle between current accel normal and reference normal.
    // This is an unsigned angle in [0..180], typically [0..90] for inclinometer use.
    if (g_imu.ref_valid) {
      const float nCur = safe_norm3(ax, ay, az);
      const float nRef = safe_norm3(g_imu.ref_ax_g, g_imu.ref_ay_g, g_imu.ref_az_g);
      if (nCur > 0.0f && nRef > 0.0f) {
        float c = (ax * g_imu.ref_ax_g + ay * g_imu.ref_ay_g + az * g_imu.ref_az_g) / (nCur * nRef);
        c = clampf(c, -1.0f, 1.0f);
        g_imu.rel_angle_deg = rad2deg(acosf(c));
      }
    } else {
      g_imu.rel_angle_deg = 0.0f;
    }
    portEXIT_CRITICAL(&g_imuMux);
  }
}

// ---- UI objects ----
static lv_obj_t *ui_mode = nullptr;
static lv_obj_t *ui_primary = nullptr;
static lv_obj_t *ui_secondary = nullptr;
static lv_obj_t *ui_debug = nullptr;
static lv_obj_t *ui_rel_zero_btn = nullptr;

static lv_obj_t *ui_batt_outline = nullptr;
static lv_obj_t *ui_batt_fill = nullptr;
static lv_obj_t *ui_batt_cap = nullptr;
static lv_obj_t *ui_batt_text = nullptr;

static lv_obj_t *ui_incl_container = nullptr;
static lv_obj_t *ui_incl_bubble = nullptr;
static lv_obj_t *ui_incl_cross_h = nullptr;
static lv_obj_t *ui_incl_cross_v = nullptr;
static lv_obj_t *ui_rel_angle_big = nullptr;

static AppMode ui_last_mode = (AppMode)255;
static bool ui_last_bg_is_red = false;

static void ui_set_mode_label();

static const lv_font_t *ui_rel_angle_font()
{
  return &lv_font_roboto_mono_48;
}

static void ui_update_background(bool relRefValid)
{
  // In REL mode: red background until zero/reference is set, then black.
  // In ABS mode: always black.
  const bool wantRed = (g_mode == AppMode::RelAngle) && !relRefValid;
  if (wantRed == ui_last_bg_is_red) return;
  ui_last_bg_is_red = wantRed;
  lv_obj_set_style_bg_color(lv_scr_act(), wantRed ? lv_palette_main(LV_PALETTE_RED) : lv_color_black(), 0);
}

static void do_calibration_action()
{
  // Tap to calibrate/zero depending on current mode.
  switch (g_mode) {
    case AppMode::RelAngle:
      imu_set_reference_normal_now();
      tone(beePin, 330); // mi
      delay(45);
      tone(beePin, 440); // ra
      delay(45);
      noTone(beePin);
      break;
    case AppMode::AbsAngle:
      break;
  }
}

static void ui_tap_cb(lv_event_t *e)
{
  if (lv_event_get_code(e) != LV_EVENT_CLICKED) return;
  do_calibration_action();
}

static uint8_t voltage_to_percent(float v)
{
  // Simple Li-ion approximation (single cell). Conservative defaults.
  // 4.20V = 100%, 3.00V = 0%
  if (!isfinite(v)) return 0;
  if (v >= 4.20f) return 100;
  if (v <= 3.00f) return 0;

  // Piecewise gives a more realistic curve than pure linear.
  if (v >= 4.00f) return (uint8_t)(80 + (v - 4.00f) * (20.0f / 0.20f));   // 4.00-4.20 => 80-100
  if (v >= 3.85f) return (uint8_t)(55 + (v - 3.85f) * (25.0f / 0.15f));   // 3.85-4.00 => 55-80
  if (v >= 3.70f) return (uint8_t)(30 + (v - 3.70f) * (25.0f / 0.15f));   // 3.70-3.85 => 30-55
  if (v >= 3.50f) return (uint8_t)(10 + (v - 3.50f) * (20.0f / 0.20f));   // 3.50-3.70 => 10-30
  return (uint8_t)(0 + (v - 3.00f) * (10.0f / 0.50f));                    // 3.00-3.50 => 0-10
}

static void battery_poll_and_update()
{
  static uint32_t lastMs = 0;
  static bool lastLow = false;
  const uint32_t now = millis();
  if (now - lastMs < 1000) return;
  lastMs = now;

  const int adcValue = analogRead(kVoltageDividerPin);
  const float voltage = (float)adcValue * (kVRef / 4095.0f);
  const float actualVoltage = voltage * ((kR1 + kR2) / kR2);

  // Smooth voltage
  if (!isfinite(g_battVoltage)) {
    g_battVoltage = actualVoltage;
  } else {
    constexpr float alpha = 0.15f;
    g_battVoltage = g_battVoltage + alpha * (actualVoltage - g_battVoltage);
  }

  g_battPct = voltage_to_percent(g_battVoltage);
  const bool low = (g_battPct < 20);

  if (ui_batt_outline && ui_batt_fill) {
    // Outline size: 34x14 with 1px border, cap 3x8
    const int innerW = 34 - 2;
    const int fillW = (int)lroundf((innerW - 2) * (g_battPct / 100.0f)); // keep a small right padding
    const int clamped = (int)clampf((float)fillW, 0.0f, (float)(innerW - 2));
    lv_obj_set_width(ui_batt_fill, clamped);

    if (low != lastLow) {
      lastLow = low;
      const lv_color_t c = low ? lv_palette_main(LV_PALETTE_RED) : lv_color_white();
      lv_obj_set_style_bg_color(ui_batt_fill, c, 0);
      if (ui_batt_text) lv_obj_set_style_text_color(ui_batt_text, c, 0);
    }

    if (ui_batt_text) {
      char s[24];
      // snprintf(s, sizeof(s), "%u%%", (unsigned)g_battPct);
      snprintf(s, sizeof(s), "%.2fV", g_battVoltage);
      lv_label_set_text(ui_batt_text, s);
    }
  }
}

static void ui_set_mode_label()
{
  if (!ui_mode) return;
  switch (g_mode) {
    case AppMode::AbsAngle: lv_label_set_text(ui_mode, "ABS"); break;
    case AppMode::RelAngle: lv_label_set_text(ui_mode, "REL"); break;
  }
}

static void ui_refresh()
{
  ImuState s;
  portENTER_CRITICAL(&g_imuMux);
  s = g_imu;
  portEXIT_CRITICAL(&g_imuMux);

  ui_update_background(s.ref_valid);

  if (ui_last_mode != g_mode) {
    ui_last_mode = g_mode;
    ui_set_mode_label();
  }
  battery_poll_and_update();

  ImuUiData ui_imu;
  ui_imu.pitch_deg = s.pitch_deg;
  ui_imu.roll_deg = s.roll_deg;
  ui_imu.rel_angle_deg = s.rel_angle_deg;
  ui_imu.ref_valid = s.ref_valid;

  switch (g_mode) {
    case AppMode::AbsAngle:
      ui_pages::abs_show(ui_incl_container, ui_rel_angle_big, ui_primary, ui_secondary, ui_rel_zero_btn);
      ui_pages::abs_update(ui_primary, ui_secondary, ui_incl_container, ui_incl_bubble, ui_imu);
      if (ui_debug) lv_label_set_text(ui_debug, "");
      break;
    case AppMode::RelAngle:
      ui_pages::rel_show(ui_incl_container, ui_rel_angle_big, ui_primary, ui_secondary, ui_rel_zero_btn);
      ui_pages::rel_update(ui_rel_angle_big, ui_debug, ui_imu);
      break;
  }
}

static void ui_timer_cb(lv_timer_t * /*t*/)
{
  ui_refresh();
}

static void ui_build()
{
  lv_obj_set_style_bg_color(lv_scr_act(), lv_color_black(), 0);
  lv_obj_set_style_text_color(lv_scr_act(), lv_color_white(), 0);

  ui_mode = lv_label_create(lv_scr_act());
  lv_obj_set_style_text_font(ui_mode, &lv_font_montserrat_20, 0);
  lv_obj_align(ui_mode, LV_ALIGN_TOP_LEFT, 13, 6);
  lv_label_set_text(ui_mode, "REL");

  // Battery widget (top-right)
  ui_batt_outline = lv_obj_create(lv_scr_act());
  lv_obj_remove_style_all(ui_batt_outline);
  lv_obj_set_size(ui_batt_outline, 34, 14);
  lv_obj_align(ui_batt_outline, LV_ALIGN_TOP_RIGHT, -13, 8); // moved 5px more left
  lv_obj_set_style_border_width(ui_batt_outline, 1, 0);
  lv_obj_set_style_border_color(ui_batt_outline, lv_color_white(), 0);
  lv_obj_set_style_radius(ui_batt_outline, 2, 0);
  lv_obj_set_style_bg_opa(ui_batt_outline, LV_OPA_TRANSP, 0);

  ui_batt_fill = lv_obj_create(ui_batt_outline);
  lv_obj_remove_style_all(ui_batt_fill);
  lv_obj_set_pos(ui_batt_fill, 2, 2);
  lv_obj_set_size(ui_batt_fill, 1, 10);
  lv_obj_set_style_bg_opa(ui_batt_fill, LV_OPA_COVER, 0);
  lv_obj_set_style_bg_color(ui_batt_fill, lv_color_white(), 0);
  lv_obj_set_style_radius(ui_batt_fill, 1, 0);

  ui_batt_cap = lv_obj_create(lv_scr_act());
  lv_obj_remove_style_all(ui_batt_cap);
  lv_obj_set_size(ui_batt_cap, 3, 8);
  lv_obj_align_to(ui_batt_cap, ui_batt_outline, LV_ALIGN_OUT_RIGHT_MID, 1, 0);
  lv_obj_set_style_bg_opa(ui_batt_cap, LV_OPA_COVER, 0);
  lv_obj_set_style_bg_color(ui_batt_cap, lv_color_white(), 0);
  lv_obj_set_style_radius(ui_batt_cap, 1, 0);

  ui_batt_text = lv_label_create(lv_scr_act());
  lv_obj_set_style_text_font(ui_batt_text, &lv_font_montserrat_12, 0);
  lv_obj_align_to(ui_batt_text, ui_batt_outline, LV_ALIGN_OUT_LEFT_MID, -3, 0); // place % to the left of battery
  lv_label_set_text(ui_batt_text, "--");

  // Inclinometer widget
  ui_incl_container = lv_obj_create(lv_scr_act());
  lv_obj_remove_style_all(ui_incl_container);
  lv_obj_set_size(ui_incl_container, 100, 100);
  lv_obj_align(ui_incl_container, LV_ALIGN_CENTER, 0, -20);
  lv_obj_set_style_border_width(ui_incl_container, 1, 0);
  lv_obj_set_style_border_color(ui_incl_container, lv_color_white(), 0);
  lv_obj_set_style_radius(ui_incl_container, 50, 0);
  lv_obj_set_style_bg_opa(ui_incl_container, LV_OPA_TRANSP, 0);

  ui_incl_cross_h = lv_obj_create(ui_incl_container);
  lv_obj_remove_style_all(ui_incl_cross_h);
  lv_obj_set_size(ui_incl_cross_h, 100, 1);
  lv_obj_set_pos(ui_incl_cross_h, 0, 49);
  lv_obj_set_style_bg_opa(ui_incl_cross_h, LV_OPA_50, 0);
  lv_obj_set_style_bg_color(ui_incl_cross_h, lv_color_white(), 0);

  ui_incl_cross_v = lv_obj_create(ui_incl_container);
  lv_obj_remove_style_all(ui_incl_cross_v);
  lv_obj_set_size(ui_incl_cross_v, 1, 100);
  lv_obj_set_pos(ui_incl_cross_v, 49, 0);
  lv_obj_set_style_bg_opa(ui_incl_cross_v, LV_OPA_50, 0);
  lv_obj_set_style_bg_color(ui_incl_cross_v, lv_color_white(), 0);

  ui_incl_bubble = lv_obj_create(ui_incl_container);
  lv_obj_remove_style_all(ui_incl_bubble);
  const int bubble_d = 12;
  const int bubble_r = bubble_d / 2;
  lv_obj_set_size(ui_incl_bubble, bubble_d, bubble_d);
  lv_obj_set_style_radius(ui_incl_bubble, bubble_r, 0);
  lv_obj_set_style_bg_opa(ui_incl_bubble, LV_OPA_COVER, 0);
  lv_obj_set_style_bg_color(ui_incl_bubble, lv_palette_main(LV_PALETTE_GREEN), 0);
  lv_obj_set_pos(ui_incl_bubble, 50 - bubble_r, 50 - bubble_r);

  // REL mode: big angle readout in place of inclinometer (same position).
  ui_rel_angle_big = lv_label_create(lv_scr_act());
  lv_obj_set_style_text_font(ui_rel_angle_big, ui_rel_angle_font(), 0);
  lv_obj_set_style_text_color(ui_rel_angle_big, lv_color_white(), 0);
  lv_obj_align(ui_rel_angle_big, LV_ALIGN_CENTER, 0, -20);
  lv_label_set_text(ui_rel_angle_big, "");
  lv_obj_add_flag(ui_rel_angle_big, LV_OBJ_FLAG_HIDDEN);

  // Text readouts
  ui_primary = lv_label_create(lv_scr_act());
  lv_obj_set_style_text_font(ui_primary, &lv_font_montserrat_20, 0);
  lv_obj_align(ui_primary, LV_ALIGN_CENTER, 0, 60);
  lv_label_set_text(ui_primary, "Initializing...");

  ui_secondary = lv_label_create(lv_scr_act());
  lv_obj_set_style_text_font(ui_secondary, &lv_font_montserrat_20, 0);
  lv_obj_align(ui_secondary, LV_ALIGN_CENTER, 0, 84);
  lv_label_set_text(ui_secondary, "");

  ui_debug = lv_label_create(lv_scr_act());
  lv_obj_set_style_text_font(ui_debug, &lv_font_montserrat_12, 0);
  lv_obj_align(ui_debug, LV_ALIGN_BOTTOM_MID, 0, -8);
  lv_label_set_text(ui_debug, "");

  // REL mode: bottom-half "Tap to set zero" button (orange)
  ui_rel_zero_btn = lv_btn_create(lv_scr_act());
  lv_obj_set_size(ui_rel_zero_btn, LCD_WIDTH, LCD_HEIGHT / 3);
  lv_obj_align(ui_rel_zero_btn, LV_ALIGN_BOTTOM_MID, 0, 0);
  lv_obj_set_style_bg_opa(ui_rel_zero_btn, LV_OPA_COVER, 0);
  lv_obj_set_style_bg_color(ui_rel_zero_btn, lv_palette_main(LV_PALETTE_ORANGE), 0);
  lv_obj_set_style_radius(ui_rel_zero_btn, 0, 0);
  lv_obj_set_style_border_width(ui_rel_zero_btn, 0, 0);
  lv_obj_clear_flag(ui_rel_zero_btn, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_add_event_cb(ui_rel_zero_btn, ui_tap_cb, LV_EVENT_CLICKED, nullptr);

  lv_obj_t *btn_label = lv_label_create(ui_rel_zero_btn);
  lv_obj_set_style_text_font(btn_label, &lv_font_montserrat_20, 0);
  lv_obj_set_style_text_color(btn_label, lv_color_white(), 0);
  lv_label_set_text(btn_label, "Tap to set zero");
  lv_obj_center(btn_label);

  // Reflect initial mode immediately (ui_refresh will keep it updated).
  if (g_mode == AppMode::AbsAngle) lv_obj_add_flag(ui_rel_zero_btn, LV_OBJ_FLAG_HIDDEN);
}

static void setup_hardware()
{
  // Keep the board powered for soft-sleep. The PWR button asserts SYS_OUT,
  // and we wake via the ESP sleep GPIO wakeup source.
  pinMode(kSysEnPin, OUTPUT);
  digitalWrite(kSysEnPin, HIGH);

  // SYS_OUT is LOW while the user presses PWR.
  pinMode(kSysOutPin, INPUT_PULLUP);

  // Mode button (BOOT): active-low to GND.
  pinMode(kModePin, INPUT_PULLUP);
  pinMode(kImuIntPin, INPUT_PULLUP);

  pinMode(kVoltageDividerPin, INPUT);

  gfx->begin();
  backlight_pwm_init();
  backlight_set(kBacklightFull);

  pinMode(beePin, OUTPUT);  // avoids accidentally driving buzzer transistor
  digitalWrite(beePin, LOW);

  WiFi.mode(WIFI_OFF);
  btStop();

  if (g_imuIntSem == nullptr) {
    g_imuIntSem = xSemaphoreCreateBinary();
  }
  if (g_imuIntSem != nullptr) {
    // Drain any stale token before enabling IRQ.
    while (xSemaphoreTake(g_imuIntSem, 0) == pdTRUE) {}
  }
  attachInterrupt(digitalPinToInterrupt(kImuIntPin), imu_int_isr, FALLING);

  Wire.begin(IIC_SDA, IIC_SCL);

  while (CST816T->begin() == false) {
    USBSerial.println("CST816T initialization fail");
    delay(500);
  }
  CST816T->IIC_Write_Device_State(
    CST816T->Arduino_IIC_Touch::Device::TOUCH_DEVICE_INTERRUPT_MODE,
    CST816T->Arduino_IIC_Touch::Device_Mode::TOUCH_DEVICE_INTERRUPT_PERIODIC
  );
  USBSerial.println("CST816T initialization success");
}

static void setup_lvgl()
{
  lv_init();
#if LV_USE_LOG != 0
  lv_log_register_print_cb(my_print);
#endif

  lv_disp_draw_buf_init(&draw_buf, buf, NULL, LCD_WIDTH * LCD_HEIGHT / 10);

  static lv_disp_drv_t disp_drv;
  lv_disp_drv_init(&disp_drv);
  disp_drv.hor_res = LCD_WIDTH;
  disp_drv.ver_res = LCD_HEIGHT;
  disp_drv.flush_cb = my_disp_flush;
  disp_drv.draw_buf = &draw_buf;
  lv_disp_drv_register(&disp_drv);

  // Touch input device (CST816T)
  static lv_indev_drv_t indev_drv;
  lv_indev_drv_init(&indev_drv);
  indev_drv.type = LV_INDEV_TYPE_POINTER;
  indev_drv.read_cb = my_touchpad_read;
  lv_indev_drv_register(&indev_drv);

  const esp_timer_create_args_t lvgl_tick_timer_args = {
    .callback = &lvgl_tick_cb,
    .arg = nullptr,
    .dispatch_method = ESP_TIMER_TASK,
    .name = "lvgl_tick",
    .skip_unhandled_events = true,
  };
  esp_timer_handle_t lvgl_tick_timer = nullptr;
  esp_timer_create(&lvgl_tick_timer_args, &lvgl_tick_timer);
  esp_timer_start_periodic(lvgl_tick_timer, EXAMPLE_LVGL_TICK_PERIOD_MS * 1000);

  ui_build();
  // UI refresh ~15 Hz.
  lv_timer_create(ui_timer_cb, kUiFramePeriodMs, nullptr);
}

static void setup_tasks()
{
  // Start IMU task
  xTaskCreatePinnedToCore(imu_task, "imu_task", 4096, nullptr, 2, &g_imuTaskHandle, 1);

  // Initialize inactivity timers.
  g_lastUserActivityMs = millis();
  g_lastMotionActivityMs = g_lastUserActivityMs;
}

void setup()
{
  USBSerial.begin(115200);

  setup_hardware();
  play_welcome_sound();
  g_bootMs = millis();
  setup_lvgl();
  setup_tasks();

  USBSerial.println("Setup complete");
}

void loop()
{
  beep_poll();
  mode_button_poll();

  // Throttle LVGL work to reduce CPU. ~30 Hz max for LVGL timers/animations.
  lv_timer_handler();
  delay(kUiFramePeriodMs);
  backlight_idle_manager();
  power_sleep_manager();
}

