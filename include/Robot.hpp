bool homeMotor(int stp, int dir, int end);

int prepPins(void);

bool moveDeg(int deg, int stp, int dir);

typedef struct Motor
{
    int step_pin;
    int dir_pin;
};
