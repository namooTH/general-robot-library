#pragma once
#include <POP32.h>
#include "rawMotor.h"

struct MotorSet {
    const int left;
    const int right;

    int left_speed = 0;
    int right_speed = 0;
    
    int left_offset = 0;
    int right_offset = 0;

    void move(int pow, double direction, bool use_offset = true) {
        direction = constrain(direction, -1.0, 1.0);

        int maxPow = abs(pow);

        int left_speed  = pow - (2.0 * pow * direction);
        int right_speed = pow + (2.0 * pow * direction);

        left_speed  = constrain(left_speed,  -maxPow, maxPow);
        right_speed = constrain(right_speed, -maxPow, maxPow);

        if (use_offset) {
            raw_motor(left, left_speed + left_offset);
            raw_motor(right, right_speed + right_offset);
        } else {
            raw_motor(left, left_speed);
            raw_motor(right, right_speed);
        }
    }


    void stop() {
        left_speed = 0;
        right_speed = 0;

        motor(left, 0);
        motor(right, 0);
    }
};