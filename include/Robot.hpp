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
    int base;
    int arm;
    int z_axis;
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
    int x;
    int y;
    int z;
};