#pragma once
#include <cmath>
#include <cstdint>

namespace battery_plug {
enum class State { Unknown, Plugged, Unplugged };

// Voltage-edge and sustained-trend heuristic, not VBUS/charge-status sensing. Call at ~1 Hz.
class Detector {
public:
    static constexpr float kTrendDeltaV = 0.004f; // Front/back 10-s means, 20 s apart.
    static constexpr float kRiseV = 0.020f;
    static constexpr float kFallV = 0.020f;
    static constexpr uint32_t kConfirmMs = 3000;
    static constexpr uint32_t kCooldownMs = 5000;

    State state() const { return state_; }
    State update(float volts, uint32_t now) {
        if (!std::isfinite(volts) || volts < 2.5f || volts > 4.5f) {
            *this = Detector{};
            return state_;
        }
        if (started_ && uint32_t(now - lastMs_) < 750) return state_;
        if (started_ && uint32_t(now - lastMs_) > 2500) *this = Detector{};
        started_ = true;
        lastMs_ = now;
        raw_[rawIndex_] = volts;
        rawIndex_ = (rawIndex_ + 1) % 3;
        if (rawCount_ < 3) ++rawCount_;
        if (rawCount_ < 3) return state_;
        const float v = median(raw_[0], raw_[1], raw_[2]);
        const State trend = updateTrend(v);
        float lo = history_[0], hi = history_[0], reference = 0;
        for (float h : history_) {
            lo = std::fmin(lo, h);
            hi = std::fmax(hi, h);
            reference += h * 0.125f;
        }
        const bool stableReference = hi - lo <= 0.040f;
        const bool ready = historyCount_ == 8;
        history_[historyIndex_] = v;
        historyIndex_ = (historyIndex_ + 1) % 8;
        if (historyCount_ < 8) ++historyCount_;
        if (!ready) return state_;
        if (cooling_ && uint32_t(now - eventMs_) < kCooldownMs) return state_;
        cooling_ = false;
        if (candidate_ == State::Unknown && trend != State::Unknown && trend != state_) {
            state_ = trend;
            resetTrend();
        }

        if (candidate_ != State::Unknown) {
            const float delta = v - candidateBase_;
            const bool persists = candidate_ == State::Plugged
                ? delta >= kRiseV * 0.75f : delta <= -kFallV * 0.75f;
            if (persists) {
                if (uint32_t(now - candidateMs_) >= kConfirmMs) {
                    state_ = candidate_;
                    resetTrend();
                    candidate_ = State::Unknown;
                    cooling_ = true;
                    eventMs_ = now;
                }
                return state_;
            }
            candidate_ = State::Unknown;
        }
        if (!stableReference) return state_;
        const float delta = v - reference;
        const State edge = delta >= kRiseV ? State::Plugged
            : delta <= -kFallV ? State::Unplugged : State::Unknown;
        if (edge != State::Unknown && edge != state_) {
            candidate_ = edge;
            candidateBase_ = reference;
            candidateMs_ = now;
        }
        return state_;
    }
private:
    void resetTrend() {
        trendCount_ = trendIndex_ = trendConfirm_ = 0;
        trendCandidate_ = State::Unknown;
    }
    float trimmedMean(unsigned start) const {
        float values[10];
        for (unsigned i = 0; i < 10; ++i)
            values[i] = trend_[(trendIndex_ + start + i) % 30];
        for (unsigned i = 1; i < 10; ++i) {
            const float v = values[i];
            unsigned j = i;
            while (j && values[j - 1] > v) { values[j] = values[j - 1]; --j; }
            values[j] = v;
        }
        float sum = 0;
        for (unsigned i = 2; i < 8; ++i) sum += values[i];
        return sum / 6.0f; // Reject short spikes at either end of the window.
    }
    State updateTrend(float v) {
        trend_[trendIndex_] = v;
        trendIndex_ = (trendIndex_ + 1) % 30;
        if (trendCount_ < 30) ++trendCount_;
        if (trendCount_ < 30) return State::Unknown;
        const float delta = trimmedMean(20) - trimmedMean(0);
        const State direction = delta >= kTrendDeltaV ? State::Plugged
            : delta <= -kTrendDeltaV ? State::Unplugged : State::Unknown;
        if (direction == State::Unknown) {
            trendCandidate_ = direction;
            trendConfirm_ = 0;
        } else if (direction != trendCandidate_) {
            trendCandidate_ = direction;
            trendConfirm_ = 1;
        } else if (trendConfirm_ < 3) {
            ++trendConfirm_;
        }
        return trendConfirm_ >= 3 ? trendCandidate_ : State::Unknown;
    }
    float trend_[30] = {};
    unsigned trendCount_ = 0, trendIndex_ = 0, trendConfirm_ = 0;
    State trendCandidate_ = State::Unknown;
    static float median(float a, float b, float c) {
        if (a > b) { const float t = a; a = b; b = t; }
        if (b > c) { const float t = b; b = c; c = t; }
        return a > b ? a : b;
    }
    State state_ = State::Unknown, candidate_ = State::Unknown;
    bool started_ = false, cooling_ = false;
    uint32_t lastMs_ = 0, candidateMs_ = 0, eventMs_ = 0;
    float raw_[3] = {}, history_[8] = {}, candidateBase_ = 0;
    unsigned rawIndex_ = 0, rawCount_ = 0, historyIndex_ = 0, historyCount_ = 0;
};
} // namespace battery_plug
