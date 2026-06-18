#pragma once
#include "sensor.h"

String get_str_from_pos_to_delimiter(int *pos, String raw_data) {
    int start = *pos;
    int split = raw_data.indexOf(';', start);

    if (split == -1) {
        *pos = raw_data.length();
        return raw_data.substring(start);
    }

    *pos = split + 1;
    return raw_data.substring(start, split);
}

// not using sstream beacuse it cannot be used with the arduino's string implementation
template <size_t N>
void parse_sensor_data(String raw_data, Sensor (*sensors)[N]) { 
    int pos = 0;
    while (pos < raw_data.length()) {
        String portStr  = get_str_from_pos_to_delimiter(&pos, raw_data);
        String whiteStr = get_str_from_pos_to_delimiter(&pos, raw_data);
        String blackStr = get_str_from_pos_to_delimiter(&pos, raw_data);
        
        int port = portStr.toInt();
        int white = whiteStr.toInt();
        int black = blackStr.toInt();
        
        for (auto &sensor : *sensors) {
            if (sensor.channel == port) {
                sensor.whiteValue = white;
                sensor.blackValue = black;
                break;
            }
        };
    }
}
