#pragma once
#include "pins.h"
#include <Arduino.h>
#include <NewPing.h>

// Master switch for the ultrasonic sensor. False means queryDistance() never
// touches the hardware at all.
//
// This exists because NewPing::ping() is a blocking busy-wait, and a sensor that
// doesn't answer costs the FULL timeout — _maxEchoTime + MAX_SENSOR_DELAY, about
// 12.75 ms at MAX_DISTANCE 120 — every single call. That is longer than
// CONTROL_LOOP_PERIOD_US, so with a silent sensor the esp_timer callback can
// never finish a tick before the next is due, the timer task starves IDLE0, and
// task_wdt aborts the firmware ~5 s in. A working sensor at ~50 cm returns in
// ~2.9 ms and hides the problem entirely, which is why this only bites when the
// sonar is unplugged or broken.
//
// Set true again once the sensor is wired and answering. When you do, note that
// ROTATING_TIL_ROCK and HABITAT_FIND each ping once per tick from AI, so the
// per-tick budget is one ping — do not add a second call site.
static constexpr bool SONAR_ENABLED = true;

class Sonar
{
    public:
        Sonar();
        static constexpr double MAX_DISTANCE = 120;

        double queryDistance();
        void begin();

    private:
        NewPing sonar;

};
