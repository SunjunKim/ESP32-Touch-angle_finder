#include "../src/battery_soc.h"
#include <cassert>
#include <cstdio>
#include <limits>

int main() {
    using battery_soc::percent;
    assert(percent(3.5f) == 0);
    assert(percent(4.1f) == 90); // Old mapping prematurely reported full here.
    assert(percent(4.2f) == 100);
    assert(percent(4.199f) < 100);
    assert(percent(4.05f) == 85);
    assert(percent(-1.0f) == 0);
    assert(percent(5.0f) == 100);
    assert(percent(std::numeric_limits<float>::quiet_NaN()) == 0);
    assert(percent(std::numeric_limits<float>::infinity()) == 0);
    unsigned previous = 0;
    for (int mv = 2500; mv <= 4500; ++mv) {
        const auto value = percent(mv / 1000.0f);
        assert(value >= previous && value <= 100);
        previous = value;
    }
    // The same measured voltage gives the same estimate in either direction.
    previous = 100;
    for (int mv = 4500; mv >= 2500; --mv) {
        const auto value = percent(mv / 1000.0f);
        assert(value <= previous);
        previous = value;
    }
    using battery_soc::Mode;
    battery_soc::Estimator e;
    uint32_t now = 1000;
    float previousSoc = e.update(3.85f, Mode::Discharging, now);
    assert(previousSoc == 50);
    assert(battery_soc::curve(4.0f, Mode::Charging) < battery_soc::curve(4.0f, Mode::Discharging));
    for (int cycle = 0; cycle < 3; ++cycle) {
        const float savedDischarge = previousSoc;
        now += 1000;
        assert(e.update(4.0f, Mode::Charging, now) == previousSoc);
        assert(e.lastDischargingSoc() == savedDischarge);
        for (int i = 0; i < 100; ++i) {
            now += 1000;
            const float next = e.update(4.0f + i * .001f, Mode::Charging, now);
            assert(std::fabs(next - previousSoc) <= .201f);
            assert(next >= 0 && next <= 100);
            previousSoc = next;
        }
        const float savedCharge = previousSoc;
        now += 1000;
        assert(e.update(3.85f, Mode::Discharging, now) == previousSoc);
        assert(e.lastChargingSoc() == savedCharge);
        for (int i = 0; i < 100; ++i) {
            now += 1000;
            const float next = e.update(3.85f - i * .001f, Mode::Discharging, now);
            assert(std::fabs(next - previousSoc) <= .201f);
            previousSoc = next;
        }
    }
    now += 1000;
    assert(e.update(NAN, Mode::Charging, now) == previousSoc);
    now += 1000;
    assert(e.update(3.8f, Mode::Unknown, now) == previousSoc);
    now += 3600000;
    assert(std::fabs(e.update(4.2f, Mode::Unknown, now) - previousSoc) <= .401f);
    battery_soc::Estimator wrap;
    const float initial = wrap.update(3.8f, Mode::Discharging, UINT32_MAX - 500);
    assert(std::fabs(wrap.update(3.5f, Mode::Discharging, 499) - initial) <= .201f);
    std::puts("battery_soc curves and continuity tests passed");
}
