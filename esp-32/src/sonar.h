#pragma once
#include "pins.h"
#include <Arduino.h>
#include <NewPing.h>

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
