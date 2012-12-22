/*
 * emontx.cpp
 *
 *  Created on: 14 Dec 2012
 *      Author: jack
 */

#ifdef TESTING
#include <tests/FakeArduino.h>
#include <stdint.h>
typedef uint8_t byte;
#else
#include <Arduino.h>
#endif // TESTING

#include "Manager.h"

Manager manager;

void setup()
{
    Serial.begin(115200);
    Serial.println(F("EmonTX"));
    manager.init();
    Serial.println(F("finished init"));
}

void loop()
{
    manager.run();
}

/* This main function would be inserted automatically by
 * the Arduino IDE but is necessary to explicitly put it here
 * for developers who aren't using the Arduino IDE. */
int main(void)
{
    init();
    setup();

    while(true) {
        loop();
    }
}
