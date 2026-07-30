#include "pins.h"
#include "sonar.h"
#include <Arduino.h>
#include <NewPing.h>

Sonar::Sonar() : sonar(SONAR_TRIG, SONAR_ECHO, MAX_DISTANCE)
{

}

void Sonar::begin()
{
    // NewPing configures trig/echo pin modes in its constructor; nothing
    // else to do here. Kept for parity with the other components' emplace-
    // then-begin() pattern (see MrKrabs::setup()).
}

double Sonar::queryDistance()
{
	// Returns 0 rather than MAX_DISTANCE when disabled, matching exactly what a
	// failed ping already yields: NewPing::ping() gives 0 us on no echo, which
	// divides down to 0 cm. Callers therefore behave the same with the sensor
	// switched off as with one that never answers — no new behaviour to reason
	// about, just no 12.75 ms stall. Worth knowing that 0 cm reads as "closest
	// possible", so AI::tickHabitatFind treats it as habitat-found immediately;
	// that is pre-existing, and convenient while bench-testing the habitat legs
	// without a sensor, but it is not a safe default for competition.
	if (!SONAR_ENABLED) {
		return 0.0;
	}
	float distance = (float)sonar.ping() / US_ROUNDTRIP_CM;
    return distance;
}
