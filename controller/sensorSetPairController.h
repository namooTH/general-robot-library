#pragma once
#include "../sensor/sensorSet.h"

class SensorSetPairController {
    public:
        SensorSet *left;
        SensorSet *right;

        double get_direction() {
            double ln = left->get_normalised();
            double rn = right->get_normalised();
            
            if (ln < 0.2 && rn < 0.2) {
                return 0.0;
            }
        
            double dir = (ln - 0.5) - (rn - 0.5);
            return constrain(dir * 2.0, -1.0, 1.0);
        }
};