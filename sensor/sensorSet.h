#pragma once
#include <POP32.h>
#include "sensor.h"
#include "../sensor/color.h"

struct SensorSet {
    Sensor *left;
    Sensor *right;

    void set_white() {
        left->set_white();
        right->set_white();
    }

    void set_black() {
        left->set_black();
        right->set_black();
    }

    double get_value() {
        return (left->get_value() + right->get_value()) / 2.0f;
    }

    double get_normalised() {
        return (left->get_normalised() + right->get_normalised()) / 2.0f;
    }

    double get_direction() {
        double ln = left->get_normalised();
        double rn = right->get_normalised();
    
        double dir = ln - rn;
        return constrain(dir, -1.0, 1.0);
    }
};