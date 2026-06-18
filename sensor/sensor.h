#pragma once
#include <POP32.h>
#include "../sensor/color.h"

struct Sensor {
    int channel;
    int blackValue;
    int whiteValue;

    void set_white() {
        whiteValue = get_value(); 
    }

    void set_black() {
        blackValue = get_value(); 
    }

    inline int get_value() {
        return analog(channel);
    }
    
    inline double get_normalised() {
        return (double) (get_value() - whiteValue) / (double) (blackValue - whiteValue); 
    }
};