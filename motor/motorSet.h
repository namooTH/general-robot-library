#pragma once
#include "rawMotor.h"

struct MotorSet {
    const int left;
    const int right;

    int left_speed = 0;
    int right_speed = 0;

    MotorSet(const int left, const int right):
        left(left),
        right(right) {}

    void move(int pow, double direction) {
        direction = constrain(direction, -1.0, 1.0);

        int maxPow = abs(pow);

        int left_speed  = pow - (2.0 * pow * direction);
        int right_speed = pow + (2.0 * pow * direction);

        left_speed  = constrain(left_speed,  -maxPow, maxPow);
        right_speed = constrain(right_speed, -maxPow, maxPow);

        raw_motor(left, left_speed);
        raw_motor(right, right_speed);
    }

    void stop() {
        left_speed = 0;
        right_speed = 0;

        raw_motor(left, 0);
        raw_motor(right, 0);
    }
};