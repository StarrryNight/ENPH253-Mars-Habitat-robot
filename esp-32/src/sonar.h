#include "pins.h"
#include <Arduino.h>
#include <NewPing.h>

class Sonar
{
    public:
        Sonar();

        double queryDistance();
        void begin();



    private:

}