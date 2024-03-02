#include <Arduino.h>
#include <math.h>
#include <WiFi.h>
#include <WebServer.h>
#include "pindef.hpp"
#include "definitions.hpp"

bool homeMotor(Motor motor, int end);

int prepPins(void);

bool moveDeg(int deg, int stp, int dir);

typedef struct Motor
{
    int step_pin;
    int dir_pin;
    int uebersetzung;
};

typedef enum status_robot
{
    STANDING_STILL,
    MOVING,
    ERROR
};

typedef struct targetAngles
{
    float base;
    float arm;
};

int homingProcedure(void);

void calculateAngle(double L1, double L2, double X, double Y, double &angle1_deg, double &angle2_deg);

void inverseKinematics(double X_current, double Y_current, double X_target, double Y_target, double L1, double L2, double &delta_theta1, double &delta_theta2);

typedef struct length
{
    double L1;
    double L2;
};

typedef struct Position
{
    float x;
    float y;
    float z;
};

typedef struct Credentials
{
    String ssid;
    String password;
};

bool startServer(Credentials credentials);

void handleRoot();
void handleFormSubmit();
void startWebServerTask(void *pvParameters);

int movetoPos(Position currPos, Position targetPos, Motor motor1, Motor motor2);