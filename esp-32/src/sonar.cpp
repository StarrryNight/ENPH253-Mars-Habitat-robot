#include "pins.h"
#include "sonar.h"
#include <Arduino.h>
#include <NewPing.h>

Sonar::Sonar() : sonar(SONAR_TRIG, SONAR_ECHO, MAX_DISTANCE)
{

}

Sonar::queryDistance()
{
    unsigned int distance = sonar.ping_cm();
    if (distance > 30 || distance < 5) {
        distance = 0;
    }
    return distance;
}