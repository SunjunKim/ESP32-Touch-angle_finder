#include "../src/battery_plug.h"
#include <cassert>
#include <cstdio>
#include <limits>
using battery_plug::Detector;
using battery_plug::State;
struct Trace {
    Detector d;
    uint32_t now = 0;
    void hold(float v, int seconds) {
        for (int i = 0; i < seconds; ++i) { d.update(v, now); now += 1000; }
    }
};
int main() {
    Trace t;
    t.hold(4.2f, 20); // Boot already connected cannot establish a direction.
    assert(t.d.state() == State::Unknown);
    t.hold(4.05f, 2);
    assert(t.d.state() == State::Unknown); // Confirmation required.
    t.hold(4.05f, 8);
    assert(t.d.state() == State::Unplugged);
    t.hold(4.18f, 10);
    assert(t.d.state() == State::Plugged);
    t.hold(4.18f, 60);
    assert(t.d.state() == State::Plugged);
    t.hold(4.02f, 10);
    assert(t.d.state() == State::Unplugged);

    Trace noise;
    noise.hold(3.8f, 10);
    noise.hold(4.0f, 1); noise.hold(3.8f, 10);
    noise.hold(3.6f, 1); noise.hold(3.8f, 10);
    noise.hold(3.95f, 2); noise.hold(3.8f, 10);
    assert(noise.d.state() == State::Unknown);
    for (int i = 0; i < 200; ++i) noise.hold(3.8f + (i % 2 ? .015f : -.015f), 1);
    assert(noise.d.state() == State::Unknown);
    Trace slow;
    slow.hold(3.8f, 12);
    slow.hold(3.825f, 8); // A small step from low-current USB charging.
    assert(slow.d.state() == State::Plugged);
    slow.hold(3.825f, 10);
    slow.hold(3.8f, 8);
    assert(slow.d.state() == State::Unplugged);
    Trace ramp;
    ramp.hold(3.8f, 12);
    for (int i = 1; i <= 10; ++i) ramp.hold(3.8f + i * .005f, 1);
    ramp.hold(3.85f, 5);
    assert(ramp.d.state() == State::Plugged);
    Trace drift;
    drift.hold(3.7f, 10);
    for (int i = 0; i < 200; ++i) drift.hold(3.7f + i * .001f, 1);
    assert(drift.d.state() == State::Plugged);
    for (int i = 0; i < 200; ++i) drift.hold(3.9f - i * .001f, 1);
    assert(drift.d.state() == State::Unplugged);
    Trace gentle;
    gentle.hold(3.8f, 40);
    for (int i = 1; i <= 90; ++i) gentle.hold(3.8f + i * .0004f, 1);
    assert(gentle.d.state() == State::Plugged);
    gentle.hold(3.836f, 45);
    assert(gentle.d.state() == State::Plugged); // Flat means retain, not unplug.
    for (int i = 1; i <= 90; ++i) gentle.hold(3.836f - i * .0004f, 1);
    assert(gentle.d.state() == State::Unplugged);
    Trace flat;
    for (int i = 0; i < 180; ++i) {
        const float spike = (i % 40 < 2) ? .1f : 0;
        flat.hold(3.8f + spike, 1);
    }
    assert(flat.d.state() == State::Unknown);

    t.now += 10000; t.hold(4.2f, 10);
    assert(t.d.state() == State::Unknown); // No edge across sleep/data gaps.
    t.hold(3.9f, 10); assert(t.d.state() == State::Unplugged);
    t.d.update(std::numeric_limits<float>::quiet_NaN(), t.now);
    assert(t.d.state() == State::Unknown);
    t.hold(4.2f, 10); assert(t.d.state() == State::Unknown);
    t.hold(5.0f, 1); assert(t.d.state() == State::Unknown);

    Trace cooldown;
    cooldown.hold(3.8f, 10);
    cooldown.hold(3.95f, 5);
    assert(cooldown.d.state() == State::Plugged);
    cooldown.hold(3.8f, 3);
    assert(cooldown.d.state() == State::Plugged); // Suppress immediate chatter.
    // Too-fast duplicate polling must not complete a candidate.
    Trace fast;
    fast.hold(3.8f, 10);
    for (unsigned i = 0; i < 100; ++i) fast.d.update(4.0f, fast.now + i);
    assert(fast.d.state() == State::Unknown);

    Trace wrap;
    wrap.now = UINT32_MAX - 15000;
    wrap.hold(3.8f, 10); wrap.hold(3.95f, 10);
    assert(wrap.d.state() == State::Plugged);
    wrap.hold(3.8f, 10); assert(wrap.d.state() == State::Unplugged);
    std::puts("battery_plug traces passed");
}
