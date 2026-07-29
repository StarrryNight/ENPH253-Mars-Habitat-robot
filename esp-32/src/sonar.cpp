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
	float distance = (float)sonar.ping() / US_ROUNDTRIP_CM;
	Serial.printf("distacne = %.2f\n",distance);
    return distance;
}
