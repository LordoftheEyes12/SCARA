#include "Robot.hpp"

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
    pinMode(Motor1_Step_Pin, OUTPUT);
    pinMode(Motor1_Dir_Pin, OUTPUT);

    pinMode(Motor2_Step_Pin, OUTPUT);
    pinMode(Motor2_Dir_Pin, OUTPUT);

    pinMode(Motor3_Step_Pin, OUTPUT);
    pinMode(Motor3_Dir_Pin, OUTPUT);

    pinMode(Endstop1_Pin, INPUT);
    pinMode(Endstop2_Pin, INPUT);
    pinMode(Endstop3_Pin, INPUT);

    pinMode(LimitSwitch1_Pin, INPUT);
    pinMode(LimitSwitch2_Pin, INPUT);
    pinMode(LimitSwitch3_Pin, INPUT);

    pinMode(stepper_enable, OUTPUT);

    return 0;
}

bool moveDeg(Motor motor, float deg)
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

    double angle1 = atan2(Y, X) - acos((L1 * L1 + XY_dist * XY_dist - L2 * L2) / (2 * L1 * sqrt(XY_dist * XY_dist)));
    double angle2 = M_PI - acos((L1 * L1 + L2 * L2 - XY_dist * XY_dist) / (2 * L1 * L2));

    angle1_deg = angle1 * (180.0 / M_PI);
    angle2_deg = angle2 * (180.0 / M_PI);
}


void inverseKinematics(double X_current, double Y_current, double X_target, double Y_target, double L1, double L2, double &delta_theta1, double &delta_theta2)
{
    double theta1_current, theta2_current;
    double theta1_target, theta2_target;

    calculateAngle(L1, L2, X_current, Y_current, theta1_current, theta2_current);

    calculateAngle(L1, L2, X_target, Y_target, theta1_target, theta2_target);

    delta_theta1 = theta1_target - theta1_current;
    delta_theta2 = theta2_target - theta2_current;
}

int movetoPos(Position currPos, Position targetPos, Motor motor1, Motor motor2)
{
    double delta_theta1, delta_theta2;

    inverseKinematics(currPos.x, currPos.y, targetPos.x, targetPos.y, Arm.L1, Arm.L2, delta_theta1, delta_theta2);

    moveDeg(motor1, delta_theta1);
    moveDeg(motor2, delta_theta2);
    moveZ(targetPos.z);
    currPos.x = targetPos.x;
    currPos.y = targetPos.y;
   
    return 0;
}

bool startServer(Credentials credentials)
{
    WiFi.begin(credentials.ssid, credentials.password);
    while (WiFi.status() != WL_CONNECTED)
    {
        delay(1000);
        Serial.println("Connecting to WiFi...");
    }
    Serial.println("Connected to WiFi");
    return true;
}

int moveZ(float z)
{
    float distance = z - current_position.z;
    int steps = distance * 100;
    if (steps < 0)
    {
        digitalWrite(Motor3_Dir_Pin, LOW);
        steps = steps * (-1);
    }
    else
    {
        digitalWrite(Motor3_Dir_Pin, HIGH);
    }
    for (int i = 1; i < steps; i++)
    {
        digitalWrite(Motor3_Step_Pin, HIGH);
        delayMicroseconds(1000);
        digitalWrite(Motor3_Step_Pin, LOW);
        delayMicroseconds(1000);
    }
    current_position.z = z;
    return 0;
}

int homingProcedure(void)
{
    homeMotor(motor1, 14);
    homeMotor(motor2, 12);
    homeMotor(motor3, 13);
    return 0;
}

int startup(void * pvParameters)
{
    prepPins();
    homingProcedure();
    startServer(credentials);
    return 0;
}
