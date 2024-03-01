#include "Robot.hpp"
#include <Arduino.h>
#include <math.h>
length Arm = {200, 150};
bool homeMotor(Motor motor, int end)
{
    int stp = motor.step_pin;
    int dir = motor.dir_pin;
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

bool moveDeg(Motor motor, int deg)
{
    int stp = motor.step_pin;
    int dir = motor.dir_pin;
    int multiplier = motor.uebersetzung;
    float steps_fl = (float)((1200 * deg * multiplier) / 360);
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

    void calculateAngle(double L1, double L2, double X, double Y, double &angle1_deg, double &angle2_deg)
    {
        double XY_dist = sqrt(X * X + Y * Y);

        // Law of cosines
        double angle1 = atan2(Y, X) - acos((L1 * L1 + XY_dist * XY_dist - L2 * L2) / (2 * L1 * sqrt(XY_dist * XY_dist)));
        double angle2 = M_PI - acos((L1 * L1 + L2 * L2 - XY_dist * XY_dist) / (2 * L1 * L2));

        // Convert angles to degrees
        angle1_deg = angle1 * (180.0 / M_PI);
        angle2_deg = angle2 * (180.0 / M_PI);
    }

    // Function to calculate the change in angles from current to target positions
    void inverseKinematics(double X_current, double Y_current, double X_target, double Y_target, double L1, double L2, double &delta_theta1, double &delta_theta2)
    {
        double theta1_current, theta2_current;
        double theta1_target, theta2_target;

        // Calculate angles for current position
        calculateAngle(L1, L2, X_current, Y_current, theta1_current, theta2_current);

        // Calculate angles for target position
        calculateAngle(L1, L2, X_target, Y_target, theta1_target, theta2_target);

        // Calculate the change in angles
        delta_theta1 = theta1_target - theta1_current;
        delta_theta2 = theta2_target - theta2_current;
    }

    int movetoPos(Position currPos, Position targetPos, Motor motor1, Motor motor2)
    {
        double delta_theta1, delta_theta2;

        // Calculate the change in angles
        inverseKinematics(currPos.x, currPos.y, targetPos.x, targetPos.y, Arm.L1, Arm.L2, delta_theta1, delta_theta2);

        // Move the motors to the new position
        moveDeg(motor1, delta_theta1);
        moveDeg(motor2, delta_theta2);

        return 0;
    }