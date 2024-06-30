#include <math.h>
#include <stdio.h>

#define MAX_AX 0.5f
#define MAX_NEG_AX 10.0f
#define MAX_NEG_AX 10.0f

#define TOF_PERIOD (1.0f / 15.0f)

static float previous_vx = 0;

float limit_velocity(float x_vel_in) {
    if ((x_vel_in > previous_vx) &&
        ((x_vel_in - previous_vx) > (MAX_AX * TOF_PERIOD))) {
        x_vel_in = previous_vx + (MAX_AX * TOF_PERIOD);
    }

    if ((x_vel_in < previous_vx) &&
        ((previous_vx - x_vel_in) > (MAX_NEG_AX * TOF_PERIOD))) {
        x_vel_in = previous_vx - (MAX_NEG_AX * TOF_PERIOD);
    }
    previous_vx = x_vel_in;

    return x_vel_in;
}
