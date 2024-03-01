#include "Robot.hpp"
#include <Arduino.h>
bool homeMotor(int stp, int dir, int end)
{
    digitalWrite(dir, LOW);
    while (!digitalRead(end))
    {

        digitalWrite(stp, HIGH);
        delayMicroseconds(500);
        digitalWrite(stp, LOW);
        delayMicroseconds(500);
    }
    return true;
}

int prepPins(void)
{
    pinMode(2, OUTPUT);
    pinMode(15, OUTPUT);
    pinMode(4, OUTPUT);
    pinMode(5, OUTPUT);
    pinMode(18, OUTPUT);
    pinMode(19, OUTPUT);
    return 0;
}

bool moveDeg(int deg, int stp, int dir)
{
    float steps_fl = (float)((1200 * deg) / 360);
    if (steps_fl < 0)
    {
        digitalWrite(dir, LOW);
        steps_fl = steps_fl * (-1);
    }
    else
    {
        digitalWrite(dir, HIGH);
    }
    Serial.print(steps_fl);
    for (int i = 1; i < (int)steps_fl; i++)
    {
        digitalWrite(stp, HIGH);
        delayMicroseconds(1000);
        digitalWrite(stp, LOW);
        delayMicroseconds(1000);
    }
    return true;
}