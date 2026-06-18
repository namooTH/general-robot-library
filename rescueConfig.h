#define RESCUE_CONFIG

#include <POP32.h>
#include "controller/sensorSetPairController.h"
#include "controller/motorSetPairController.hpp"
#include "draw/draw.hpp"
#include "sensor/IMUSensor.h"
#include "sensor/parser.h"

const MotorSet motorSets[2] = { {1, 3, 0, 0, 0, 3},   // Front
                                {2, 4, 0, 0, 0, 3}};  // Back

Sensor sensors[4] = {
    {8}, {5},  // Front
    {6}, {7},  // Back
};

// Sensor sensors[8] = {
//     {4}, {2},  // Front
//     {5}, {3},  // Back
//     {6}, {8},  // Left
//     {0}, {1}   // Right
// };

const String sensor_data = "8;3852;1124;5;1123;159;6;1914;356;7;3414;624";

// SensorSet sensorSets[4] = { { &sensors[0], &sensors[1] },   // Front
//                             { &sensors[2], &sensors[3] },   // Back
//                             { &sensors[4], &sensors[5] },   // Left
//                             { &sensors[6], &sensors[7] } }; // Right

SensorSet sensorSets[2] = { { &sensors[0], &sensors[1] },   // Front
                            { &sensors[2], &sensors[3] } }; // Back

// Optional<SensorSetPairController> sensor_controller({ &sensorSets[2], &sensorSets[3] });
Optional<SensorSetPairController> sensor_controller;

IMUSensor imu_sensor;
MotorSetPairController  motor_controller  = { &sensor_controller,
                                              &imu_sensor,
                                              
                                              sensorSets[0],
                                              sensorSets[1],
  
                                              motorSets[0], 
                                              motorSets[1] };

String sensor_debug_names[4] = {
    "Front Sensor",
    "Back Sensor",
    "Left Sensor",
    "Right Sensor"
};

void debug_sensor() {
    size_t sensor_idx = 0;

    clear();
    sensor_idx = 0;
    drawTextFmt(0, 0, WHITE, "White");
    for (SensorSet &sensor_set: sensorSets) {
        drawTextFmt(0, 10+(10*sensor_idx), WHITE, "%d        %d", sensor_set.left->whiteValue, sensor_set.right->whiteValue);
        sensor_idx++;
    }
    flip();
    while (!SW_A());
    while (SW_A());

    clear();
    sensor_idx = 0;
    drawTextFmt(0, 0, WHITE, "Black");
    for (SensorSet &sensor_set: sensorSets) {
        drawTextFmt(0, 10+(10*sensor_idx), WHITE, "%d        %d", sensor_set.left->blackValue, sensor_set.right->blackValue);
        sensor_idx++;
    }
    flip();
    while (!SW_A());
}

void cali_sensors() {
    size_t sensor_idx = 0;
    for (SensorSet &sensor_set: sensorSets) {
        while (!SW_A()) {
            clear();
            drawTextFmt(0, 0, WHITE, "White");
            drawTextFmt(0, 10, WHITE, sensor_debug_names[sensor_idx].c_str());
            drawTextFmt(0, 30, WHITE, "%d        %d", sensor_set.left->get_value(), sensor_set.right->get_value());
            flip();
        }
        sensor_set.set_white();
        while (SW_A());
        sensor_idx++;
    }

    sensor_idx = 0;
    for (SensorSet &sensor_set: sensorSets) {
        while (!SW_A()) {
            clear();
            drawTextFmt(0, 0, WHITE, "Black");
            drawTextFmt(0, 10, WHITE, sensor_debug_names[sensor_idx].c_str());
            drawTextFmt(0, 30, WHITE, "%d        %d", sensor_set.left->get_value(), sensor_set.right->get_value());
            flip();
        }
        sensor_set.set_black();
        while (SW_A());
        sensor_idx++;
    }

    clear();
    drawTextFmt(0, 0, WHITE, "Plug in USB to");
    drawTextFmt(0, 10, WHITE, "get values");
    flip();
    while (!SW_A());
    while (SW_A());

    String cali_data = "";
    for (SensorSet &sensor_set: sensorSets) {
        char buffer[64];
        snprintf(buffer, sizeof(buffer), ";%d;%d;%d;%d;%d;%d", sensor_set.left->channel, sensor_set.left->whiteValue, sensor_set.left->blackValue, sensor_set.right->channel, sensor_set.right->whiteValue, sensor_set.right->blackValue);
        cali_data += buffer;
    }
    cali_data = cali_data.substring(1); // cut the first ; off
    SerialUSB.print(cali_data.c_str());

    debug_sensor();
}

struct RescueInit {
    RescueInit() {
        parse_sensor_data(sensor_data, &sensors);
        imu_sensor.Init();
    };
};

static RescueInit rescue_init;