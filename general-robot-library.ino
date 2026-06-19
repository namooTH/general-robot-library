#include "rescueConfig.h"
#include "draw/draw.hpp"
#include "draw/menu.hpp"
#include "sensor/color.h"

void setup() {
  asm(".global _printf_float");
  SerialUSB.begin(9600);
  Wire.setClock(1000000);
  OLED_DMA_Init();
  imu_sensor.Init();

  delay(20);
  servo(1, 175);
  deflag();
}

void deploy_dice() {
  servo(1, 55);
  delay(300);
  servo(1, 180);
  delay(300);
  servo(1, 150);
}

void deflag() {
  servo(2, 180);
}

void flag() {
  servo(2, 80);
}


// 90 L
// -90 R
void run() {
  clear();
  drawText(20, 20, "RESETTING IMU...", WHITE);
  drawText(30, 40, "DO NOT MOVE", WHITE);
  drawText(35, 50, "THE ROBOT", WHITE);
  flip();
  delay(1000);
  imu_sensor.Reset();
  imu_sensor.calibrateIMU();
  clear();
  flip();

  motor_controller.forward(153, 600);
  motor_controller.turnDegreeFront(-90);
  motor_controller.forward(153, 100);
  motor_controller.turnDegreeFront(0);
  motor_controller.forward(153, 200);
  motor_controller.turnDegreeFront(90);
  motor_controller.forward(153, 200);
  // deploy
  motor_controller.forward(153, 200);

  motor_controller.backward(153, 600);
  motor_controller.rotate_to(180);

  motor_controller.stop();
}
 

Menu tests = { { { "Test Motor", []() {
                    while (1) motor_controller.move(100, 0.0);
                  } },
                 { "Test IMU", []() {
                    while (1) {
                      clear();
                      drawTextFmt(0, 0, WHITE, "%f", imu_sensor.getYaw());
                      drawTextFmt(0, 10, WHITE, "SW_OK to Reset");
                      flip();
                      if (SW_OK()) {
                        imu_sensor.Reset();
                      }
                    }
                  } },
                 { "Test Dice", deploy_dice },
                 { "Test Flag", flag },
                 { "Test DeFlag", deflag },
                 { "Sensor Calibrated Data", debug_sensor },
                 { "Align", []() {
                    motor_controller.align(true);
                  } },
                 { "Test Rotate 90", []() {
                    motor_controller.imu_sensor->Reset();
                    motor_controller.rotate_to(-90.0);
                  } },
                 { "Test Rotate 180", []() {
                    motor_controller.imu_sensor->Reset();
                    motor_controller.rotate_to(180.0);
                  } },
                 { "Run Until Black", []() {
                    motor_controller.run_until_black();
                  } },
                 { "Run Until Black Backward", []() {
                    motor_controller.run_until_black(0.0, true, true);
                  } } } };

Menu menu = { { { "Run", run },
                { "Speedrun", [](){
                  motor_controller.run_until_white();
                  flag();
                  motor_controller.stop();
                }},
                { "Tests", []() {
                   tests.menu();
                 } },
                { "Calibrate Sensors", cali_sensors } } };

void loop() {
  menu.menu();
}