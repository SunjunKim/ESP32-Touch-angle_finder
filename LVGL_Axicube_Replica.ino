// Build target and validated dependency versions are documented in README.md.

#include <stdint.h>

// Full definition must appear before any function that uses AppMode::Calibrate
// (e.g. power_sleep_manager). A forward declaration alone hides enumerators.
#include "src/pages/app_mode.h"

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
#include "src/pages/cal_page.h"
#include "src/pages/page_types.h"
#include "src/pages/app_hooks.h"
#include "src/pages/imu_math.h"
#include "src/pages/imu_state.h"
#include "src/battery_soc.h"
#include "src/battery_plug.h"

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
static constexpr uint32_t kInactivityScrOffAfterMs = 30'000;
static constexpr uint32_t kMotionActivityHoldMs = 2'000; // keep "active" briefly after motion

// ---- Power-off ("sleep") ----
// Requirements:
// - 15s of inactivity -> enter ESP sleep (wake via SYS_OUT / PWR button)
// - Long-press PWR (SYS_OUT LOW while pressed) -> enter ESP sleep
static constexpr uint32_t kInactivitySleepAfterMs = 600'000;  // 10 minutes
static constexpr uint32_t kPwrLongPressMs = 2'000;
static constexpr uint32_t kPwrDebounceMs = 50;
static constexpr uint32_t kPwrIgnoreAfterBootMs = 3'000;
static constexpr uint32_t kPwrReleaseWaitTimeoutMs = 5'000;
static constexpr uint32_t kPwrIgnoreAfterWakeMs = 1'500;

// Below this smoothed cell voltage, force the same power-off path as manual sleep
// (SYS_EN low + light sleep fallback).
static constexpr float kBattForceSleepBelowV = 3.5f;
static constexpr uint32_t kBattForceSleepConfirmMs = 2'000;

// Battery state (updated in battery_poll_and_update). Defined here so power_sleep_manager() sees it.
static float g_battVoltage = NAN;
static uint8_t g_battPct = 0;

static constexpr uint8_t kBacklightFull = 255; // 0..255
static constexpr uint8_t kBacklightDim  = 25;  // tweak to taste (lower = darker, less power)
static constexpr uint8_t kBacklightOff  = 1;  // tweak to taste (lower = darker, less power)

static constexpr uint32_t kBacklightPwmHz = 5000;
static constexpr uint8_t  kBacklightPwmResBits = 8; // duty 0..255
static constexpr int      kBacklightPwmChannel = 0; // legacy cores only (kept for reference)

static const char kAppVersion[] = "v0.0.2";

static uint8_t g_backlightDuty = kBacklightFull;
static uint32_t g_lastUserActivityMs = 0;
static uint32_t g_lastMotionActivityMs = 0;
/** After waking from kBacklightOff, drop the whole touch sequence until release so LVGL sees no click. */
static bool g_touch_swallow_until_release = false;

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
  const bool shouldTurnOff = !motionRecentlyActive && idleForMs >= kInactivityScrOffAfterMs;
  const bool shouldDim = !motionRecentlyActive && idleForMs >= kInactivityDimAfterMs;

  uint8_t target = kBacklightFull;
  if (shouldTurnOff) {
    target = kBacklightOff;
  } else if (shouldDim) {
    target = kBacklightDim;
  }
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

  // ---- Critical battery: force sleep below threshold ----
  // Runs before boot/wake grace so a depleted pack is not kept awake by those delays.
  static uint32_t s_lowBattSinceMs = 0;
  if (isfinite(g_battVoltage) && g_battVoltage < kBattForceSleepBelowV) {
    if (s_lowBattSinceMs == 0) {
      s_lowBattSinceMs = now;
    } else if ((now - s_lowBattSinceMs) >= kBattForceSleepConfirmMs) {
      s_lowBattSinceMs = 0;
      enter_poweroff_sleep("low battery", /*wait_for_pwr_release=*/false);
    }
  } else {
    s_lowBattSinceMs = 0;
  }

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
  if (app_mode_snapshot() == AppMode::Calibrate) {
    return;
  }
  const uint32_t lastAny = (g_lastMotionActivityMs > g_lastUserActivityMs) ? g_lastMotionActivityMs : g_lastUserActivityMs;
  if (lastAny != 0 && (now - lastAny) >= kInactivitySleepAfterMs) {
    enter_poweroff_sleep("inactivity", /*wait_for_pwr_release=*/false);
  }
}

#if LVGL_VERSION_MAJOR >= 9
static uint16_t buf[LCD_WIDTH * LCD_HEIGHT / 10];
#else
static lv_disp_draw_buf_t draw_buf;
static lv_color_t buf[LCD_WIDTH * LCD_HEIGHT / 10];
#endif

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

#if LVGL_VERSION_MAJOR >= 9
static void my_disp_flush(lv_display_t *disp, const lv_area_t *area, uint8_t *color_p)
#else
static void my_disp_flush(lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *color_p)
#endif
{
  const uint32_t w = (area->x2 - area->x1 + 1);
  const uint32_t h = (area->y2 - area->y1 + 1);

#if (LV_COLOR_16_SWAP != 0)
  gfx->draw16bitBeRGBBitmap(area->x1, area->y1, (uint16_t *)color_p, w, h);
#else
  gfx->draw16bitRGBBitmap(area->x1, area->y1, (uint16_t *)color_p, w, h);
#endif

  lv_disp_flush_ready(disp);
}

static void lvgl_tick_cb(void * /*arg*/)
{
  lv_tick_inc(EXAMPLE_LVGL_TICK_PERIOD_MS);
}

// ---- App: modes ----
static AppMode g_mode = AppMode::RelAngle;
static constexpr uint8_t kAppModeCount = 3;
static portMUX_TYPE g_appModeMux = portMUX_INITIALIZER_UNLOCKED;

static inline AppMode next_mode(AppMode m)
{
  return (AppMode)(((uint8_t)m + 1u) % kAppModeCount);
}

AppMode app_mode_snapshot()
{
  portENTER_CRITICAL(&g_appModeMux);
  const AppMode m = g_mode;
  portEXIT_CRITICAL(&g_appModeMux);
  return m;
}

void app_mark_user_activity()
{
  mark_user_activity();
}

void app_start_tone(uint16_t hz, uint32_t durationMs)
{
  start_beep(hz, durationMs);
}

void app_play_rel_zero_sound()
{
  tone(beePin, 330);
  delay(45);
  tone(beePin, 440);
  delay(45);
  noTone(beePin);
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
    portENTER_CRITICAL(&g_appModeMux);
    g_mode = next_mode(g_mode);
    portEXIT_CRITICAL(&g_appModeMux);
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

#if LVGL_VERSION_MAJOR >= 9
static void my_touchpad_read(lv_indev_t *, lv_indev_data_t *data)
#else
static void my_touchpad_read(lv_indev_drv_t *, lv_indev_data_t *data)
#endif
{
  const bool screenAsleep = (g_backlightDuty == kBacklightOff);

  if (CST816T->IIC_Interrupt_Flag == true) {
    CST816T->IIC_Interrupt_Flag = false;
    mark_user_activity();

    if (screenAsleep) {
      g_touch_swallow_until_release = true;
      data->state = LV_INDEV_STATE_REL;
      return;
    }
    if (g_touch_swallow_until_release) {
      data->state = LV_INDEV_STATE_REL;
      return;
    }

    data->state = LV_INDEV_STATE_PR;
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
    if (g_touch_swallow_until_release) {
      g_touch_swallow_until_release = false;
    }
  }
}

// ---- Battery (from 07_LVGL_Measuring_voltage) ----
static constexpr int kVoltageDividerPin = 1; // GPIO1
static constexpr float kVRef = 3.3f;
static constexpr float kR1 = 143000.0f;
static constexpr float kR2 = 65000.0f;

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

ImuState g_imu;
portMUX_TYPE g_imuMux = portMUX_INITIALIZER_UNLOCKED;
#ifndef AXICUBE_REL_DIAGNOSTICS
#define AXICUBE_REL_DIAGNOSTICS 0
#endif

static void imu_task(void * /*param*/)
{
  // Accelerometer-only REL; no gyro startup calibration or integration.
  qmi.setPins(kImuIntPin);
  if (!qmi.begin(Wire, QMI8658_L_SLAVE_ADDRESS, IIC_SDA, IIC_SCL)) {
    portENTER_CRITICAL(&g_imuMux);
    g_imu.inited = false;
    portEXIT_CRITICAL(&g_imuMux);
    vTaskDelete(NULL);
  }

  // Accelerometer only: no gyro COD or bias settling.
  qmi.disableGyroscope();
  // "Stable" choice: modest ODR + LPF enabled.
  qmi.configAccelerometer(
    SensorQMI8658::ACC_RANGE_2G,
    SensorQMI8658::ACC_ODR_125Hz,
    SensorQMI8658::LPF_MODE_0
  );
  qmi.enableAccelerometer();

  qmi.enableLockingMechanism();
  qmi.enableINT(SensorQMI8658::INTERRUPT_PIN_1);

  // INT-driven IMU reads: wait for data-ready interrupt.
  // Keep a timeout as a resilience fallback.
  const TickType_t intWait = pdMS_TO_TICKS(4);
  float prev_ax = 0.0f, prev_ay = 0.0f, prev_az = 0.0f;
  bool prev_valid = false;
  bool prev_mode_was_calibrate = false;
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

    const AppMode modeSnap = app_mode_snapshot();
    const bool in_calibrate = (modeSnap == AppMode::Calibrate);
    ui_pages::cal_wizard_imu_sync(in_calibrate);

    if (!qmi.getDataReady()) {
      continue;
    }

    float ax, ay, az;
    if (!qmi.getAccelerometer(ax, ay, az)) {
      continue;
    }

    if (in_calibrate) {
      portENTER_CRITICAL(&g_imuMux);
      g_imu.ref_valid=g_imu.sample_valid=false;
      portEXIT_CRITICAL(&g_imuMux);
      ui_pages::cal_wizard_imu_sample(ax, ay, az);
      mark_user_activity();
      if (!prev_valid) {
        prev_ax = ax;
        prev_ay = ay;
        prev_az = az;
        prev_valid = true;
        mark_motion_activity();
      } else {
        const float delta = imu_math::safe_norm3(ax - prev_ax, ay - prev_ay, az - prev_az);
        if (delta >= kMotionDeltaG) {
          mark_motion_activity();
        }
        prev_ax = ax;
        prev_ay = ay;
        prev_az = az;
      }
      prev_mode_was_calibrate = true;
      continue;
    }

    if (prev_mode_was_calibrate) {
      prev_valid = false;
      prev_mode_was_calibrate = false;
    }

    ui_pages::ImuCalibratedSample imu_cal;
    ui_pages::imu_calibrated_reading(ax, ay, az, &imu_cal, true);

    const float pitch = imu_cal.pitch_deg;
    const float roll = imu_cal.roll_deg;
    const float tilt = imu_cal.tilt_deg;

    const float mx = imu_cal.ax_g;
    const float my = imu_cal.ay_g;
    const float mz = imu_cal.az_g;

    if (!prev_valid) {
      prev_ax = mx;
      prev_ay = my;
      prev_az = mz;
      prev_valid = true;
      mark_motion_activity();
    } else {
      const float delta = imu_math::safe_norm3(mx - prev_ax, my - prev_ay, mz - prev_az);
      if (delta >= kMotionDeltaG) {
        mark_motion_activity();
      }
      prev_ax = mx;
      prev_ay = my;
      prev_az = mz;
    }

    portENTER_CRITICAL(&g_imuMux);
    g_imu.ax_g = mx;
    g_imu.ay_g = my;
    g_imu.az_g = mz;
    g_imu.pitch_deg = pitch;
    g_imu.roll_deg = roll;
    g_imu.tilt_deg = tilt;
    portEXIT_CRITICAL(&g_imuMux);

    ui_pages::rel_update_angle_sample();
    // Use the same motion flag as the red angle indicator to wake the screen
    // and refresh inactivity timers, including small angle changes.
    portENTER_CRITICAL(&g_imuMux);
    const bool angleMoving = g_imu.moving;
    portEXIT_CRITICAL(&g_imuMux);
    if (angleMoving) {
      mark_motion_activity();
    }
#if AXICUBE_REL_DIAGNOSTICS
    static uint32_t diag_ms=0;
    if(USBSerial&&(uint32_t)(millis()-diag_ms)>=1000){
      diag_ms=millis();ImuState snapshot;
      portENTER_CRITICAL(&g_imuMux);snapshot=g_imu;portEXIT_CRITICAL(&g_imuMux);
      char line[256];
      int length=snprintf(line,sizeof(line),"REL_CONTACT A=%.6f,%.6f,%.6f tilt=%.4f ref=%.4f angle=%.4f set=%d valid=%d moving=%d normal=%u gyro=off\n",
        mx,my,mz,snapshot.abs_angle_deg,snapshot.ref_angle,snapshot.rel_angle_deg,
        snapshot.ref_valid,snapshot.sample_valid,snapshot.moving,(unsigned)snapshot.ref_normal_axis);
      if(length>0&&length<(int)sizeof(line)&&USBSerial.availableForWrite()>=length)
        USBSerial.write((const uint8_t *)line,length);
    }
#endif
  }
}

// ---- UI objects ----
static lv_obj_t *ui_mode = nullptr;
static lv_obj_t *ui_primary = nullptr;
static lv_obj_t *ui_secondary = nullptr;
static lv_obj_t *ui_debug = nullptr;
static lv_obj_t *ui_rel_zero_btn = nullptr;
static lv_obj_t *ui_cal_tap_layer = nullptr;

static lv_obj_t *ui_batt_outline = nullptr;
static lv_obj_t *ui_batt_fill = nullptr;
static lv_obj_t *ui_batt_cap = nullptr;
static lv_obj_t *ui_batt_text = nullptr;
static lv_obj_t *ui_batt_plug = nullptr;

static lv_obj_t *ui_incl_container = nullptr;
static lv_obj_t *ui_incl_bubble = nullptr;
static lv_obj_t *ui_incl_cross_h = nullptr;
static lv_obj_t *ui_incl_cross_v = nullptr;
static lv_obj_t *ui_rel_angle_big = nullptr;
static lv_obj_t *ui_splash = nullptr;

static AppMode ui_last_mode = (AppMode)255;
static uint8_t ui_last_bg_kind = 255; // 0=black, 1=rel-warn, 2=cal

static void ui_set_mode_label(AppMode mode);

static void ui_update_background(bool relRefValid)
{
  uint8_t kind = 0;
  if (g_mode == AppMode::Calibrate) {
    kind = 2;
  } else if ((g_mode == AppMode::RelAngle) && !relRefValid) {
    kind = 1;
  }
  if (kind == ui_last_bg_kind) {
    return;
  }
  ui_last_bg_kind = kind;
  lv_color_t bg = lv_color_black();
  if (kind == 2) {
    bg = lv_color_hex(0x0a1520);
  } else if (kind == 1) {
    bg = lv_color_hex(0x333333);
  }
  lv_obj_set_style_bg_color(lv_scr_act(), bg, 0);
}

static void battery_poll_and_update()
{
  static battery_plug::Detector plugDetector;
  static battery_soc::Estimator socEstimator;
  static uint32_t lastMs = 0;
  static int8_t lastBattColorState = -1; // Low/full color plus external-power bit.
  const uint32_t now = millis();
  if (now - lastMs < 1000) return;
  lastMs = now;

  const int adcValue = analogRead(kVoltageDividerPin);
  const float voltage = (float)adcValue * (kVRef / 4095.0f);
  const float actualVoltage = voltage * ((kR1 + kR2) / kR2);

  // Use pre-EMA voltage so the SOC filter does not hide plug/unplug edges.
  const auto plugState = plugDetector.update(actualVoltage, now);
  // USB host SOF traffic works without an open serial monitor. Power-only
  // chargers still use voltage inference; host presence is not charge status.
  const bool showChargeIcon = HWCDC::isPlugged() ||
      plugState == battery_plug::State::Plugged;
  if (ui_batt_plug) {
    if (showChargeIcon)
      lv_obj_clear_flag(ui_batt_plug, LV_OBJ_FLAG_HIDDEN);
    else
      lv_obj_add_flag(ui_batt_plug, LV_OBJ_FLAG_HIDDEN);
  }

  // Smooth voltage
  if (!isfinite(g_battVoltage)) {
    g_battVoltage = actualVoltage;
  } else {
    constexpr float alpha = 0.15f;
    g_battVoltage = g_battVoltage + alpha * (actualVoltage - g_battVoltage);
  }

  const auto socMode = plugState == battery_plug::State::Plugged
      ? battery_soc::Mode::Charging
      : plugState == battery_plug::State::Unplugged
          ? battery_soc::Mode::Discharging : battery_soc::Mode::Unknown;
  g_battPct = static_cast<uint8_t>(lroundf(socEstimator.update(g_battVoltage, socMode, now)));
  const bool low = (g_battPct < 20);
  const bool full = (g_battPct > 95);

  if (ui_batt_outline && ui_batt_fill) {
    // Outline size: 34x14 with 1px border, cap 3x8
    const int innerW = 34 - 2;
    const int fillW = (int)lroundf((innerW - 2) * (g_battPct / 100.0f)); // keep a small right padding
    const int clamped = (int)imu_math::clampf((float)fillW, 0.0f, (float)(innerW - 2));
    lv_obj_set_width(ui_batt_fill, clamped);

    const int8_t battColorState = (low ? 1 : (full ? 2 : 0)) + (showChargeIcon ? 4 : 0);
    if (battColorState != lastBattColorState) {
      lastBattColorState = battColorState;
      const lv_color_t c = low
                           ? lv_palette_lighten(LV_PALETTE_RED, 2)
                           : (full ? lv_palette_lighten(LV_PALETTE_GREEN, 2) : lv_color_white());
      // A muted teal fill keeps the white bolt legible at every charge level.
      lv_obj_set_style_bg_color(ui_batt_fill,
          showChargeIcon ? lv_color_hex(0x267C69) : c, 0);
      const lv_color_t edge = showChargeIcon ? lv_color_hex(0x82DCC0) : lv_color_white();
      lv_obj_set_style_border_color(ui_batt_outline, edge, 0);
      if (ui_batt_cap) lv_obj_set_style_bg_color(ui_batt_cap, edge, 0);
      if (ui_batt_text) lv_obj_set_style_text_color(ui_batt_text, c, 0);
    }

    if (ui_batt_text) {
      char s[24];
      snprintf(s, sizeof(s), "%.2fV", g_battVoltage);
      lv_label_set_text(ui_batt_text, s);
      lv_obj_align_to(ui_batt_text, ui_batt_outline, LV_ALIGN_OUT_LEFT_MID, -6, 0);
    }
  }
}

static void ui_set_mode_label(AppMode mode)
{
  if (!ui_mode) return;
  switch (mode) {
    case AppMode::AbsAngle: lv_label_set_text(ui_mode, "ABS"); break;
    case AppMode::RelAngle: lv_label_set_text(ui_mode, "REL"); break;
    case AppMode::Calibrate: lv_label_set_text(ui_mode, "CAL"); break;
  }
}

static void ui_refresh()
{
  bool imu_ready = false;
  portENTER_CRITICAL(&g_imuMux);
  imu_ready = g_imu.inited;
  portEXIT_CRITICAL(&g_imuMux);

  if (ui_splash) {
    if (!imu_ready) {
      lv_obj_clear_flag(ui_splash, LV_OBJ_FLAG_HIDDEN);
      lv_obj_move_foreground(ui_splash);
      return;
    }
    lv_obj_add_flag(ui_splash, LV_OBJ_FLAG_HIDDEN);
  }

  const AppMode mode = app_mode_snapshot();
  const AppMode prev_mode = ui_last_mode;
  ImuState s;
  portENTER_CRITICAL(&g_imuMux);
  s = g_imu;
  portEXIT_CRITICAL(&g_imuMux);

  ui_update_background(s.ref_valid);

  if (ui_last_mode != mode) {
    if (mode == AppMode::Calibrate && prev_mode != AppMode::Calibrate) {
      ui_pages::cal_invalidate_label_caches();
    }
    ui_last_mode = mode;
    ui_set_mode_label(mode);
  }
  battery_poll_and_update();

  ImuUiData ui_imu;
  ui_imu.pitch_deg = s.pitch_deg;
  ui_imu.roll_deg = s.roll_deg;
  ui_imu.tilt_deg = s.tilt_deg;
  ui_imu.rel_angle_deg = s.rel_angle_deg;
  ui_imu.ref_valid = s.ref_valid;
  ui_imu.ref_normal_axis = s.ref_normal_axis;
  ui_imu.sample_valid=s.sample_valid&&s.sample_ms&&(uint32_t)(millis()-s.sample_ms)<500;
  ui_imu.moving=s.moving;

  switch (mode) {
    case AppMode::AbsAngle:
      ui_pages::cal_hide_edges();
      if (ui_cal_tap_layer != nullptr) {
        lv_obj_add_flag(ui_cal_tap_layer, LV_OBJ_FLAG_HIDDEN);
      }
      ui_pages::abs_show(ui_incl_container, ui_rel_angle_big, ui_primary, ui_secondary, ui_rel_zero_btn);
      ui_pages::abs_update(ui_primary, ui_secondary, ui_incl_container, ui_incl_bubble, ui_imu);
      if (ui_debug) lv_label_set_text(ui_debug, "");
      break;
    case AppMode::RelAngle:
      ui_pages::cal_hide_edges();
      if (ui_cal_tap_layer != nullptr) {
        lv_obj_add_flag(ui_cal_tap_layer, LV_OBJ_FLAG_HIDDEN);
      }
      ui_pages::rel_show(ui_incl_container, ui_rel_angle_big, ui_primary, ui_secondary, ui_rel_zero_btn);
      ui_pages::rel_update(ui_rel_angle_big, ui_debug, ui_rel_zero_btn, ui_imu);
      break;
    case AppMode::Calibrate: {
      const CalUiData cu = ui_pages::cal_ui_for_refresh();
      ui_pages::cal_show(ui_incl_container, ui_rel_angle_big, ui_primary, ui_secondary, ui_debug, ui_rel_zero_btn);
      ui_pages::cal_update(ui_primary, ui_secondary, ui_debug, cu);
      if (ui_cal_tap_layer != nullptr) {
        if (cu.awaiting_tap) {
          lv_obj_clear_flag(ui_cal_tap_layer, LV_OBJ_FLAG_HIDDEN);
          lv_obj_move_foreground(ui_cal_tap_layer);
        } else {
          lv_obj_add_flag(ui_cal_tap_layer, LV_OBJ_FLAG_HIDDEN);
        }
      }
      break;
    }
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
  lv_obj_align(ui_mode, LV_ALIGN_TOP_LEFT, 17, 6);
  lv_label_set_text(ui_mode, "REL");

  // Battery widget (top-right)
  ui_batt_outline = lv_obj_create(lv_scr_act());
  lv_obj_remove_style_all(ui_batt_outline);
  lv_obj_set_size(ui_batt_outline, 34, 14);
  lv_obj_align(ui_batt_outline, LV_ALIGN_TOP_RIGHT, -17, 8); // moved 5px more left
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

  // Transparent white bolt overlays the battery, without a rectangular badge.
  ui_batt_plug = lv_label_create(lv_scr_act());
  lv_label_set_text(ui_batt_plug, LV_SYMBOL_CHARGE);
  lv_obj_set_style_text_font(ui_batt_plug, &lv_font_montserrat_12, 0);
  lv_obj_set_style_text_color(ui_batt_plug, lv_color_white(), 0);
  lv_obj_set_style_bg_opa(ui_batt_plug, LV_OPA_TRANSP, 0);
  lv_obj_clear_flag(ui_batt_plug, LV_OBJ_FLAG_CLICKABLE);
  lv_obj_align_to(ui_batt_plug, ui_batt_outline, LV_ALIGN_CENTER, 0, 0);
  lv_obj_add_flag(ui_batt_plug, LV_OBJ_FLAG_HIDDEN);

  ui_batt_cap = lv_obj_create(lv_scr_act());
  lv_obj_remove_style_all(ui_batt_cap);
  lv_obj_set_size(ui_batt_cap, 3, 8);
  lv_obj_align_to(ui_batt_cap, ui_batt_outline, LV_ALIGN_OUT_RIGHT_MID, 1, 0);
  lv_obj_set_style_bg_opa(ui_batt_cap, LV_OPA_COVER, 0);
  lv_obj_set_style_bg_color(ui_batt_cap, lv_color_white(), 0);
  lv_obj_set_style_radius(ui_batt_cap, 1, 0);

  ui_batt_text = lv_label_create(lv_scr_act());
  lv_obj_set_style_text_font(ui_batt_text, &lv_font_montserrat_12, 0);
  lv_obj_align_to(ui_batt_text, ui_batt_outline, LV_ALIGN_OUT_LEFT_MID, -6, 0); // place voltage to the left of battery
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

  ui_rel_angle_big = ui_pages::rel_create_angle_label(lv_scr_act());

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

  ui_rel_zero_btn = ui_pages::rel_create_zero_button(lv_scr_act());
  ui_cal_tap_layer = ui_pages::cal_create_intro_tap_layer(lv_scr_act());

  // Reflect initial mode immediately (ui_refresh will keep it updated).
  if (g_mode == AppMode::AbsAngle || g_mode == AppMode::Calibrate) {
    lv_obj_add_flag(ui_rel_zero_btn, LV_OBJ_FLAG_HIDDEN);
  }

  // Splash until IMU task finishes init + gyro COD calibration (g_imu.inited).
  ui_splash = lv_obj_create(lv_scr_act());
  lv_obj_remove_style_all(ui_splash);
  lv_obj_set_size(ui_splash, LCD_WIDTH, LCD_HEIGHT);
  lv_obj_align(ui_splash, LV_ALIGN_TOP_LEFT, 0, 0);
  lv_obj_set_style_bg_color(ui_splash, lv_color_black(), 0);
  lv_obj_set_style_bg_opa(ui_splash, LV_OPA_COVER, 0);
  lv_obj_clear_flag(ui_splash, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_add_flag(ui_splash, LV_OBJ_FLAG_CLICKABLE);

  lv_obj_t *splash_col = lv_obj_create(ui_splash);
  lv_obj_remove_style_all(splash_col);
  lv_obj_set_width(splash_col, LCD_WIDTH);
  lv_obj_set_height(splash_col, LV_SIZE_CONTENT);
  lv_obj_align(splash_col, LV_ALIGN_CENTER, 0, -(LCD_HEIGHT / 8));
  lv_obj_set_layout(splash_col, LV_LAYOUT_FLEX);
  lv_obj_set_flex_flow(splash_col, LV_FLEX_FLOW_COLUMN);
  lv_obj_set_flex_align(splash_col, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
  lv_obj_set_style_pad_row(splash_col, 10, 0);
  lv_obj_set_style_pad_top(splash_col, 8, 0);
  lv_obj_set_style_pad_bottom(splash_col, 8, 0);

  // No transform_zoom here: with LV_COLOR_SCREEN_TRANSP 0 (typical Arduino LVGL), SW draw
  // cannot create the alpha layer and the label is not drawn at all.
  lv_obj_t *splash_title = lv_label_create(splash_col);
  lv_obj_set_width(splash_title, LCD_WIDTH);
  lv_label_set_long_mode(splash_title, LV_LABEL_LONG_WRAP);
  lv_obj_set_style_text_align(splash_title, LV_TEXT_ALIGN_CENTER, 0);
  lv_obj_set_style_text_font(splash_title, &lv_font_montserrat_28, 0);
  lv_label_set_text(splash_title, "[InoCube]");

  lv_obj_t *splash_ver = lv_label_create(splash_col);
  lv_obj_set_width(splash_ver, LCD_WIDTH);
  lv_label_set_long_mode(splash_ver, LV_LABEL_LONG_WRAP);
  lv_obj_set_style_text_align(splash_ver, LV_TEXT_ALIGN_CENTER, 0);
  lv_obj_set_style_text_font(splash_ver, &lv_font_montserrat_20, 0);
  lv_label_set_text(splash_ver, kAppVersion);

  lv_obj_move_foreground(ui_splash);
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

  (void)ui_pages::cal_load_accel_prefs();
}

static void setup_lvgl()
{
  lv_init();
#if LV_USE_LOG != 0
  lv_log_register_print_cb(my_print);
#endif

#if LVGL_VERSION_MAJOR >= 9
  lv_display_t *disp=lv_display_create(LCD_WIDTH,LCD_HEIGHT);
  lv_display_set_color_format(disp,LV_COLOR_FORMAT_RGB565);
  lv_display_set_flush_cb(disp,my_disp_flush);
  lv_display_set_buffers(disp,buf,nullptr,sizeof(buf),LV_DISPLAY_RENDER_MODE_PARTIAL);
  lv_indev_t *indev=lv_indev_create();
  lv_indev_set_type(indev,LV_INDEV_TYPE_POINTER);
  lv_indev_set_read_cb(indev,my_touchpad_read);
#else
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
#endif

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
  USBSerial.setTxTimeoutMs(0);
  USBSerial.setTxBufferSize(1024);
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
  ui_pages::cal_flush_pending_beep();
  beep_poll();
  ui_pages::cal_poll_delayed_beep();
  beep_poll();
  mode_button_poll();

  // Throttle LVGL work to reduce CPU. ~30 Hz max for LVGL timers/animations.
  lv_timer_handler();
  delay(kUiFramePeriodMs);
  backlight_idle_manager();
  power_sleep_manager();
}
