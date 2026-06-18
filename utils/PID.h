#pragma once

struct PID {
    float kp, ki, kd;
    float integral = 0;
    float prevErr  = 0;

    float update(float error, float dt) {
        integral += error * dt;
        float derivative = (error - prevErr) / dt;
        prevErr = error;
        return kp * error + ki * integral + kd * derivative;
    }

    void reset() {
        integral = 0;
        prevErr = 0;
    }
};

struct YawPID {
    float kp, ki, kd;
    float integral = 0;
    float prevMeasurement = 0;
    bool firstUpdate = true;

    float update(float error, float measurement, float dt) {
        integral += error * dt;
        
        float derivative = 0;
        if (!firstUpdate) {
            derivative = -(measurement - prevMeasurement) / dt;
        }
        
        prevMeasurement = measurement;
        firstUpdate = false;
        
        float p_term = kp * error;
        float i_term = ki * integral;
        float d_term = kd * derivative;
        float output = p_term + i_term + d_term;
        
        // Debug print
        SerialUSB.print("E:");
        SerialUSB.print(error);
        SerialUSB.print(" dt:");
        SerialUSB.print(dt, 4);
        SerialUSB.print(" P:");
        SerialUSB.print(p_term);
        SerialUSB.print(" I:");
        SerialUSB.print(i_term);
        SerialUSB.print(" D:");
        SerialUSB.print(d_term);
        SerialUSB.print(" Out:");
        SerialUSB.println(output);
        
        return output;
    }

    void reset() {
        integral = 0;
        prevMeasurement = 0;
        firstUpdate = true;
    }
};