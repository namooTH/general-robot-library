#pragma once
#include <POP32.h>

// ZX-IMU Sensor, for 'YAW' value, use Serial1 to communicate (static).
// MUST use .Init() to initialize the sensor before use.
struct IMUSensor{
    double_t cYaw=0,cPitch=0,cRoll=0;
    bool status = false;

    uint8_t rxBuf[8];

    void Reset(){
        ZeroYaw();
        ToggleAutoMode();
    }

    // Initialize the IMU sensor with 115200 baud rate.
    // WARNING: This function will take about 3050ms to complete.
    void Init(){
        Serial1.begin(115200);
        Reset();
    }
    
    void ToggleAutoMode(){
        Serial1.write(0XA5);Serial1.write(0X52);
        delay(60);
    }

    void ToggleQueryMode(){
        Serial1.write(0XA5);Serial1.write(0X51);
        delay(60);
    }

    // Set Pitch, Roll and Yaw to zero
    // WARNING: delay upto 1950ms
    inline void ZeroYaw() {
        //Zero Pitch
        Serial1.write(0XA5);Serial1.write(0X54); 
        delay(60);
        //Zero Yaw
        Serial1.write(0XA5);Serial1.write(0X55);
        delay(60);
    }

    // Repeated yaw-zero until near zero
    //
    // 'precision' : the precision to stop zeroing, default is 0.02f
    // 'time_out' : the maximum time to wait for zeroing, default is 10'000ms
    void AutoZero(float_t precision = 0.02f,int32_t time_out = 10000) {
        ZeroYaw();
        unsigned long start = HAL_GetTick();
        while (true) {
            if (fetchIMU() && fabs(cYaw) <= precision) break;
            if (HAL_GetTick() - start > time_out) {
                ZeroYaw();
                break;
            }
        }
    }

    // Fetch data from IMU sensor, return true if successful
    // pvYaw will be updated with the latest yaw value
    inline bool fetchIMU() {
        static int8_t idx = 0;
        while (Serial1.available()){
            uint8_t byte = Serial1.read();
            if (idx == 0 && byte != 0xAA) continue;
                rxBuf[idx++] = byte;
                if (idx == 8) {
                    idx = 0;
                    if (rxBuf[7] == 0x55) {
                        cYaw = (int16_t)(rxBuf[1] << 8 | rxBuf[2]) / 100.0f;
                        cPitch = (int16_t)(rxBuf[3] << 8 | rxBuf[4]) / 100.0f;
                        cRoll = (int16_t)(rxBuf[5] << 8 | rxBuf[6]) / 100.0f;
                        status = true;
                        return true;
                    }
            }
        }
        status = false;
        return false;
    }

    // Get the current yaw value
    //
    // 'do_fetch' : true - fetching new data, flase - using previous value
    inline float_t getYaw(bool do_fetch = true){
        if (do_fetch) {
            if (!fetchIMU()) return cYaw; // If fetch failed, return previous value
        }
        return cYaw;
    }

    // Log the current yaw value, status of IMU sensor, to OLED screen. 
    // this function DO NOT update, fetch the IMU sensor.
    //
    // 'doBeep' : true - beep when logging, false - no beep
    inline void LogYaw(bool doBeep = false){
        if(doBeep)beep();
        oled.clear();
        oled.text(0,0,"Yaw=%f", cYaw);
        oled.text(1,0,"Status: %s", status ? "OK" : "Error");
        oled.show();
    }
        
};