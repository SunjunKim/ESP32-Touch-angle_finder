#pragma once

#include <cmath>
#include <cstdint>

namespace battery_soc {
// Generic Li-Po model, not a calibrated fuel gauge. See README for limits.
inline uint8_t percent(float voltage)
{
    struct Point { float volts; float pct; };
    static constexpr Point curve[] = {
        {3.50f, 0}, {3.60f, 5}, {3.70f, 10}, {3.75f, 20},
        {3.80f, 35}, {3.85f, 50}, {3.90f, 60}, {3.95f, 70},
        {4.00f, 80}, {4.10f, 90}, {4.20f, 100},
    };
    if (!std::isfinite(voltage) || voltage <= curve[0].volts) return 0;
    if (voltage >= 4.20f) return 100;
    for (unsigned i = 1; i < sizeof(curve) / sizeof(curve[0]); ++i) {
        if (voltage <= curve[i].volts) {
            const auto &lo = curve[i - 1];
            const auto &hi = curve[i];
            const float pct = lo.pct + (voltage - lo.volts) *
                (hi.pct - lo.pct) / (hi.volts - lo.volts);
            // Reserve 100% for reaching the upper voltage endpoint. This still
            // means an estimate, never a charger-reported charge-done event.
            return static_cast<uint8_t>(std::fmin(99.0f, std::round(pct)));
        }
    }
    return 100;
}

enum class Mode { Unknown, Discharging, Charging };

// Initial charge-polarization allowance, to be calibrated with this cell/load.
// No current sensor: this is an empirical correction, not IR compensation.
inline float curve(float voltage, Mode mode) {
    return percent(voltage - (mode == Mode::Charging ? 0.080f : 0.0f));
}

class Estimator {
public:
    static constexpr float kMaxPercentPerSecond = 0.20f;
    static constexpr float kOffsetDecayPerSecond = 0.02f;
    float lastChargingSoc() const { return charging_.last; }
    float lastDischargingSoc() const { return discharging_.last; }
    float value() const { return value_; }

    float update(float voltage, Mode mode, uint32_t now) {
        if (!std::isfinite(voltage) || voltage < 2.5f || voltage > 4.5f)
            return value_; // Preserve continuity across a bad reading.
        const float target = curve(voltage, mode);
        if (!initialized_) {
            initialized_ = true;
            mode_ = mode;
            value_ = target;
            active().last = value_;
            lastMs_ = now;
            return value_;
        }
        // Cap elapsed time so wakeup after a long pause cannot jump the display.
        const float dt = std::fmin(uint32_t(now - lastMs_) / 1000.0f, 2.0f);
        lastMs_ = now;
        if (mode != mode_) {
            active().last = value_; // Save outgoing mode's last displayed SOC.
            const float outgoingLast = active().last;
            mode_ = mode;
            auto &incoming = active();
            // Rebase the saved incoming history onto the outgoing last SOC.
            // Never restore an old charging/discharging percentage directly.
            const float saved = std::isfinite(incoming.last) ? incoming.last : outgoingLast;
            const float bridge = outgoingLast - saved;
            incoming.last = saved + bridge;
            incoming.offset = incoming.last - target;
            value_ = incoming.last;
            return value_; // Exactly continuous on the transition sample.
        }
        auto &state = active();
        state.offset = approach(state.offset, 0, kOffsetDecayPerSecond * dt);
        const float desired = clamp(target + state.offset);
        value_ = approach(state.last, desired, kMaxPercentPerSecond * dt);
        state.last = value_;
        return value_;
    }
private:
    struct History { float last = NAN; float offset = 0; };
    History charging_, discharging_, unknown_;
    History &active() {
        return mode_ == Mode::Charging ? charging_
            : mode_ == Mode::Discharging ? discharging_ : unknown_;
    }
    static float clamp(float x) { return std::fmax(0.0f, std::fmin(100.0f, x)); }
    static float approach(float from, float to, float step) {
        return from + std::fmax(-step, std::fmin(step, to - from));
    }
    bool initialized_ = false;
    Mode mode_ = Mode::Unknown;
    float value_ = 0;
    uint32_t lastMs_ = 0;
};
} // namespace battery_soc
