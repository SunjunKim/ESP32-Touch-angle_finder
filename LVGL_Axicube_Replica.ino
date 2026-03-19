#include <stdint.h>

// Forward declaration to survive Arduino auto-prototypes.
enum class AppMode : uint8_t;

#include <Arduino.h>
#include <lvgl.h>
#include <Wire.h>
#include <esp_timer.h>
#include <esp_sleep.h>
#include <driver/gpio.h>
#include <memory>

#include "Arduino_GFX_Library.h"
#include "Arduino_DriveBus_Library.h"
#include "pin_config.h"
#include "lv_conf.h"

#include "SensorQMI8658.hpp"
#include "HWCDC.h"
#include "WiFi.h"

LV_FONT_DECLARE(lv_font_roboto_mono_48);

HWCDC USBSerial;

const int beePin = 42;     // Define GPIO42 for buzzer
static constexpr int kSysEnPin = 41; // Schematic: SYS_EN -> GPIO41, set as OUTPUT
static constexpr int kSysOutPin = 40; // Schematic: SYS_OUT -> GPIO40 -> PWR btn, set as INPUT

static constexpr int kModePin = 0; // Schematic: GPIO0 = boot / used as Mode Button

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
  // Bring up QMI8658 in a stable 6DOF configuration (acc + gyro).
  // Reason: accel-only "tilt" can appear to drift over time due to slow bias/temperature changes.
  // We instead track a gravity vector with gyro integration and slowly correct with the accelerometer
  // (a complementary fusion that is stable and resistant to bias).
  if (!qmi.begin(Wire, QMI8658_L_SLAVE_ADDRESS, IIC_SDA, IIC_SCL)) {
    portENTER_CRITICAL(&g_imuMux);
    g_imu.inited = false;
    portEXIT_CRITICAL(&g_imuMux);
    vTaskDelete(NULL);
  }

  // "Stable" choice: modest ODR + LPF enabled.
  qmi.configAccelerometer(
    SensorQMI8658::ACC_RANGE_2G,
    SensorQMI8658::ACC_ODR_62_5Hz,
    SensorQMI8658::LPF_MODE_3,
    true,
    true
  );
  qmi.enableAccelerometer();

  qmi.configGyroscope(
    SensorQMI8658::GYR_RANGE_64DPS,
    SensorQMI8658::GYR_ODR_56_05Hz,
    SensorQMI8658::LPF_MODE_3,
    true,
    true
  );
  qmi.enableGyroscope();

  // --- Startup calibration (assumes the device is stationary for a short moment) ---
  // Gyro bias dominates "drift" when integrating. Estimate it on boot.
  float gyro_bias_x = 0.0f, gyro_bias_y = 0.0f, gyro_bias_z = 0.0f;
  float acc_mean_x = 0.0f, acc_mean_y = 0.0f, acc_mean_z = 0.0f;
  int cal_n = 0;

  const uint32_t cal_start_ms = millis();
  while (millis() - cal_start_ms < 1200) {
    if (!qmi.getDataReady()) {
      vTaskDelay(pdMS_TO_TICKS(2));
      continue;
    }
    float ax, ay, az, gx, gy, gz;
    if (!qmi.getAccelerometer(ax, ay, az)) continue;
    if (!qmi.getGyroscope(gx, gy, gz)) continue;
    gyro_bias_x += gx;
    gyro_bias_y += gy;
    gyro_bias_z += gz;
    acc_mean_x += ax;
    acc_mean_y += ay;
    acc_mean_z += az;
    cal_n++;
    vTaskDelay(pdMS_TO_TICKS(2));
  }

  if (cal_n > 0) {
    const float inv = 1.0f / (float)cal_n;
    gyro_bias_x *= inv;
    gyro_bias_y *= inv;
    gyro_bias_z *= inv;
    acc_mean_x *= inv;
    acc_mean_y *= inv;
    acc_mean_z *= inv;
  }

  // Simple accel DC correction based on the stationary mean.
  // Assumption: when stationary, the *true* acceleration is gravity with magnitude 1g.
  // We estimate an additive bias = measured_mean - (measured_mean normalized to 1g).
  float acc_unit_x = acc_mean_x, acc_unit_y = acc_mean_y, acc_unit_z = acc_mean_z;
  normalize3(acc_unit_x, acc_unit_y, acc_unit_z);
  const float acc_off_x = (cal_n > 0) ? (acc_mean_x - acc_unit_x) : 0.0f;
  const float acc_off_y = (cal_n > 0) ? (acc_mean_y - acc_unit_y) : 0.0f;
  const float acc_off_z = (cal_n > 0) ? (acc_mean_z - acc_unit_z) : 0.0f;

  // Filtered accel + gravity estimate (unit vector, in sensor frame).
  float fax = 0.0f, fay = 0.0f, faz = 0.0f;
  bool seeded = false;
  float gex = 0.0f, gey = 0.0f, gez = 1.0f;

  // --- Inactivity (stand-still) motion sensitivity ---
  // "Common sense" defaults: ignore small IMU jitter; only treat as motion when the device
  // actually changes orientation or experiences a noticeable acceleration bump.
  //
  // NOTE: Higher thresholds => LESS sensitive motion detection => easier to reach "inactivity".
  static constexpr float kIdleDetectAngleDeltaDeg = 1.5f;  // degrees
  static constexpr float kIdleDetectAccMagDeltaG  = 0.12f; // g

  // Compare against a slowly-updated "still reference" to avoid noise sensitivity.
  float stillRefPitch = 0.0f;
  float stillRefRoll = 0.0f;
  float stillRefAccMag = 1.0f;
  bool stillRefValid = false;
  static constexpr float kStillRefAlpha = 0.02f; // smaller => reference changes slowly (better jitter rejection)

  // ~50 Hz sampling target (reduces power; still smooth at ~15 Hz UI).
  const TickType_t period = pdMS_TO_TICKS(20);
  TickType_t lastWake = xTaskGetTickCount();
  uint64_t last_us = esp_timer_get_time();

  portENTER_CRITICAL(&g_imuMux);
  g_imu.inited = true;
  portEXIT_CRITICAL(&g_imuMux);

  for (;;) {
    vTaskDelayUntil(&lastWake, period);

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

    // Apply coarse calibration
    gx -= gyro_bias_x;
    gy -= gyro_bias_y;
    gz -= gyro_bias_z;
    ax -= acc_off_x;
    ay -= acc_off_y;
    az -= acc_off_z;

    if (!seeded) {
      fax = ax;
      fay = ay;
      faz = az;
      seeded = true;
      // Seed gravity estimate from the first accelerometer sample.
      gex = fax;
      gey = fay;
      gez = faz;
      normalize3(gex, gey, gez);
    } else {
      // 1-pole low-pass for accel used in the fusion correction term.
      constexpr float alpha = 0.10f;
      fax = fax + alpha * (ax - fax);
      fay = fay + alpha * (ay - fay);
      faz = faz + alpha * (az - faz);
    }

    // --- Complementary fusion on gravity vector ---
    const uint64_t now_us = esp_timer_get_time();
    float dt = (now_us > last_us) ? (float)(now_us - last_us) * 1e-6f : 0.01f;
    last_us = now_us;
    dt = clampf(dt, 0.001f, 0.05f);

    // Integrate gyro: update gravity estimate by small-angle rotation.
    // Gyro units are deg/s in this library.
    const float wx = gx * 0.017453292519943295f;
    const float wy = gy * 0.017453292519943295f;
    const float wz = gz * 0.017453292519943295f;

    // dg/dt = - w x g   (gravity vector expressed in body/sensor frame)
    const float cx = (wy * gez - wz * gey);
    const float cy = (wz * gex - wx * gez);
    const float cz = (wx * gey - wy * gex);
    gex -= cx * dt;
    gey -= cy * dt;
    gez -= cz * dt;
    normalize3(gex, gey, gez);

    // Accelerometer correction (low-frequency reference).
    float axn = fax, ayn = fay, azn = faz;
    normalize3(axn, ayn, azn);
    // beta sets how quickly we trust accel for long-term leveling. Smaller = smoother, less vibration sensitivity.
    constexpr float beta = 0.1f;
    gex = (1.0f - beta) * gex + beta * axn;
    gey = (1.0f - beta) * gey + beta * ayn;
    gez = (1.0f - beta) * gez + beta * azn;
    normalize3(gex, gey, gez);

    // Compute pitch/roll from the fused gravity vector.
    // Keep the sketch's original conventions so UI behavior stays consistent.
    const float pitch = rad2deg(atan2f(gex, sqrtf(gey * gey + gez * gez)));
    const float roll  = rad2deg(atan2f(gey, sqrtf(gex * gex + gez * gez)));

    // --- Significant movement detection (for inactivity dimming) ---
    // Compare to a "still reference" (slow-moving baseline) so tiny jitter doesn't count.
    // Also include accel magnitude change to catch linear motion while staying level.
    const float accMag = safe_norm3(fax, fay, faz);
    if (!stillRefValid) {
      stillRefPitch = pitch;
      stillRefRoll = roll;
      stillRefAccMag = accMag;
      stillRefValid = true;
      mark_motion_activity();
    } else {
      const float dp = fabsf(pitch - stillRefPitch);
      const float dr = fabsf(roll - stillRefRoll);
      const float dMag = fabsf(accMag - stillRefAccMag);

      const bool isMotion = (dp > kIdleDetectAngleDeltaDeg) ||
                            (dr > kIdleDetectAngleDeltaDeg) ||
                            (dMag > kIdleDetectAccMagDeltaG);
      if (isMotion) {
        mark_motion_activity();
        // Reset baseline to current state after motion so we don't re-trigger repeatedly.
        stillRefPitch = pitch;
        stillRefRoll = roll;
        stillRefAccMag = accMag;
      } else {
        // Slowly adapt baseline while "still" (rejects drift but ignores jitter).
        stillRefPitch = stillRefPitch + kStillRefAlpha * (pitch - stillRefPitch);
        stillRefRoll  = stillRefRoll  + kStillRefAlpha * (roll  - stillRefRoll);
        stillRefAccMag = stillRefAccMag + kStillRefAlpha * (accMag - stillRefAccMag);
      }
    }

    portENTER_CRITICAL(&g_imuMux);
    // Expose the fused gravity vector (unit-ish), since that's what the angles come from.
    g_imu.ax_g = gex;
    g_imu.ay_g = gey;
    g_imu.az_g = gez;

    g_imu.pitch_deg = pitch;
    g_imu.roll_deg = roll;

    // REL mode: angle between current accel normal and reference normal.
    // This is an unsigned angle in [0..180], typically [0..90] for inclinometer use.
    if (g_imu.ref_valid) {
      const float nCur = safe_norm3(gex, gey, gez);
      const float nRef = safe_norm3(g_imu.ref_ax_g, g_imu.ref_ay_g, g_imu.ref_az_g);
      if (nCur > 0.0f && nRef > 0.0f) {
        float c = (gex * g_imu.ref_ax_g + gey * g_imu.ref_ay_g + gez * g_imu.ref_az_g) / (nCur * nRef);
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

// ---- UI caching to avoid unnecessary redraws ----
static char ui_last_line1[64] = {0};
static char ui_last_line2[64] = {0};
static char ui_last_dbg[96] = {0};
static char ui_last_rel_big[64] = {0};
static int ui_last_bubble_x = INT32_MIN;
static int ui_last_bubble_y = INT32_MIN;
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

static void set_label_if_changed(lv_obj_t *label, char *cache, size_t cacheLen, const char *next)
{
  if (!label || !next) return;
  if (strncmp(cache, next, cacheLen) == 0) return;
  strncpy(cache, next, cacheLen - 1);
  cache[cacheLen - 1] = '\0';
  lv_label_set_text(label, cache);
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
      snprintf(s, sizeof(s), "%u%%", (unsigned)g_battPct);
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

static void ui_update_inclinometer(float pitchDeg, float rollDeg)
{
  if (!ui_incl_container || !ui_incl_bubble) return;

  // Map +/- 45deg into the bubble motion range.
  const float maxDeg = 45.0f;
  const float pxRange = 40.0f; // container roughly 100x100 => bubble can move within +/-40

  const float dx = clampf(rollDeg / maxDeg, -1.0f, 1.0f) * pxRange;
  const float dy = -clampf(pitchDeg / maxDeg, -1.0f, 1.0f) * pxRange;

  const int cx = 50;
  const int cy = 50;
  const int bx = (int)lroundf(cx + dx - 8);
  const int by = (int)lroundf(cy + dy - 8);
  if (bx != ui_last_bubble_x || by != ui_last_bubble_y) {
    ui_last_bubble_x = bx;
    ui_last_bubble_y = by;
    lv_obj_set_pos(ui_incl_bubble, bx, by);
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

  if (!ui_primary || !ui_secondary) return;

  char line1[64];
  char line2[64];
  char dbg[96];
  char relBig[64];
  dbg[0] = '\0';

  switch (g_mode) {
    case AppMode::AbsAngle: {
      if (ui_incl_container) lv_obj_clear_flag(ui_incl_container, LV_OBJ_FLAG_HIDDEN);
      if (ui_rel_angle_big) lv_obj_add_flag(ui_rel_angle_big, LV_OBJ_FLAG_HIDDEN);
      if (ui_primary) lv_obj_clear_flag(ui_primary, LV_OBJ_FLAG_HIDDEN);
      if (ui_secondary) lv_obj_clear_flag(ui_secondary, LV_OBJ_FLAG_HIDDEN);
      if (ui_rel_zero_btn) lv_obj_add_flag(ui_rel_zero_btn, LV_OBJ_FLAG_HIDDEN);
      snprintf(line1, sizeof(line1), "Pitch: %.2f°", s.pitch_deg);
      snprintf(line2, sizeof(line2), "Roll : %.2f°", s.roll_deg);
      set_label_if_changed(ui_primary, ui_last_line1, sizeof(ui_last_line1), line1);
      set_label_if_changed(ui_secondary, ui_last_line2, sizeof(ui_last_line2), line2);
      ui_update_inclinometer(s.pitch_deg, s.roll_deg);
      break;
    }
    case AppMode::RelAngle: {
      if (ui_incl_container) lv_obj_add_flag(ui_incl_container, LV_OBJ_FLAG_HIDDEN);
      if (ui_rel_angle_big) lv_obj_clear_flag(ui_rel_angle_big, LV_OBJ_FLAG_HIDDEN);
      if (ui_primary) lv_obj_add_flag(ui_primary, LV_OBJ_FLAG_HIDDEN);
      if (ui_secondary) lv_obj_add_flag(ui_secondary, LV_OBJ_FLAG_HIDDEN);
      if (ui_rel_zero_btn) lv_obj_clear_flag(ui_rel_zero_btn, LV_OBJ_FLAG_HIDDEN);

      const float relRounded = round_to_step(s.rel_angle_deg, 0.1f);
      snprintf(relBig, sizeof(relBig), "%.1f°", relRounded);
      set_label_if_changed(ui_rel_angle_big, ui_last_rel_big, sizeof(ui_last_rel_big), relBig);

      snprintf(dbg, sizeof(dbg), "Ref: %s", s.ref_valid ? "SET" : "NOT SET");
      break;
    }
  }

  if (ui_debug) {
    set_label_if_changed(ui_debug, ui_last_dbg, sizeof(ui_last_dbg), dbg);
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

  pinMode(kVoltageDividerPin, INPUT);

  gfx->begin();
  backlight_pwm_init();
  backlight_set(kBacklightFull);

  pinMode(beePin, OUTPUT);  // avoids accidentally driving buzzer transistor
  digitalWrite(beePin, LOW);

  WiFi.mode(WIFI_OFF);
  btStop();

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

  // Short press on Mode button cycles app modes.
  // Debounced edge detector with a max press time to qualify as "short".
  {
    static uint8_t lastReading = HIGH;
    static uint8_t stable = HIGH;
    static uint32_t lastChangeMs = 0;
    static bool wasPressed = false;
    static uint32_t pressStartMs = 0;

    const uint32_t now = millis();
    const uint8_t reading = (uint8_t)digitalRead(kModePin);

    if (reading != lastReading) {
      lastReading = reading;
      lastChangeMs = now;
    }

    constexpr uint32_t kDebounceMs = 30;
    if ((now - lastChangeMs) >= kDebounceMs && stable != lastReading) {
      stable = lastReading;

      if (stable == LOW) {
        wasPressed = true;
        pressStartMs = now;
        mark_user_activity();
      } else {
        if (wasPressed) {
          wasPressed = false;
          const uint32_t heldMs = now - pressStartMs;
          constexpr uint32_t kShortPressMaxMs = 500;
          if (heldMs > 0 && heldMs <= kShortPressMaxMs) {
            g_mode = next_mode(g_mode);
            mark_user_activity();
            start_beep(440, 60);
          }
        }
      }
    }
  }

  // Throttle LVGL work to reduce CPU. ~30 Hz max for LVGL timers/animations.
  lv_timer_handler();
  delay(kUiFramePeriodMs);
  backlight_idle_manager();
  power_sleep_manager();
}

