#pragma once

#include "../motor/motorSet.h"
#include "sensorSetPairController.h"
#include "../sensor/IMUSensor.h"
#include "../utils/PID.h"
#include "../utils/mathHelper.h"
#include "../utils/optional.h"
#include "../draw/draw.hpp"

class MotorSetPairController {
    public:
        const float POWER_FACTOR = 1.0f;

        Optional<SensorSetPairController> *sensor_set_pair_controller;
        IMUSensor *imu_sensor;

        SensorSet front_sensor;
        SensorSet back_sensor;
        
        MotorSet front;
        MotorSet back;

        Optional<SensorSet> far_front_sensor;
        Optional<SensorSet> far_back_sensor;
        
        MotorSetPairController(Optional<SensorSetPairController>* sspc,
                               IMUSensor* imu,
                               const SensorSet& front_s,
                               const SensorSet& back_s,
                               const MotorSet& front_m,
                               const MotorSet& back_m) :
        front(front_m),
        back(back_m)
        {
            this->sensor_set_pair_controller = sspc;
            this->imu_sensor = imu;
            this->front_sensor = front_s;
            this->back_sensor = back_s;

            if (sensor_set_pair_controller->hasValue()) {
                this->far_front_sensor = {sensor_set_pair_controller->getValue().left->left, sensor_set_pair_controller->getValue().right->left};
                this->far_back_sensor = {sensor_set_pair_controller->getValue().left->right, sensor_set_pair_controller->getValue().right->right};
            }
        }

        /**
         * Move the robot using motors.
         * 
         * ARGS:
         *     int pow : How much motor power you want to use.
         *     double direction : A number in range of -1.0 to 1.0, negative means left, and positive means right.
         * 
         * RETURNS:
         *     Nothing.
         */
        void move(int pow, double direction) {
            front.move(pow, direction);
            back.move(pow, direction);
        }
        
        /**
         * Stop the robot's motors.
         * 
         * ARGS:
         *     None.
         *
         * RETURNS:
         *     Nothing.
         */
        void stop() {
            front.stop();
            back.stop();
        }
        
        /**
         * Move the robot forward for some seconds.
         *
         * ARGS:
         *     int max_speed : The maximum motor speed.
         *     float timer : Time to move in milliseconds.
         *
         * RETURNS:
         *     Nothing.
         */
        void forward(int max_speed, float timer) {
            float kp = 3.0;
            float kd = 1.0;

            unsigned long timer_forward = millis();

            float previous_error = 0;

            while (1) {
                unsigned long elapsed_time = millis() - timer_forward;
                float error = getWorldYaw() - lastPerfectYaw;
                float derivative = error - previous_error;
                float pd_value = (error * kp) + (derivative * kd);

                float direction = constrain(pd_value / max_speed, -1.0, 1.0);
                move(max_speed, direction);

                if (elapsed_time >= timer * POWER_FACTOR) {
                    stop();
                    break;
                }

                previous_error = error;
            }
        }

        /**
         * Move the robot backward for some seconds.
         *
         * ARGS:
         *     int max_speed : The maximum motor speed.
         *     float timer : Time to move in milliseconds.
         *
         * RETURNS:
         *     Nothing.
         */
        void backward(int max_speed, float timer) {
            float kp = 3.0;
            float kd = 1.0;

            unsigned long timer_forward = millis();

            float previous_error = 0;

            while (1) {
                unsigned long elapsed_time = millis() - timer_forward;
                float error = getWorldYaw() - lastPerfectYaw;
                float derivative = error - previous_error;
                float pd_value = (error * kp) + (derivative * kd);

                float direction = constrain(pd_value / max_speed, -1.0, 1.0);
                move(-max_speed, direction);

                if (elapsed_time >= timer * POWER_FACTOR) {
                    stop();
                    break;
                }

                previous_error = error;
            }
        }

        /**
         * Move forward until a sensor detects black.
         * WARNING: This method's functionality is not verified by namooTH yet.
         *
         * ARGS:
         *     float black [DEFAULT=0.9] : The threshold where it is considered "Black" when the sensor value is more than this [BAD WORDING].
         *
         * RETURNS:
         *     int : How long it takes to detect white in milliseconds.
         */
        int check_front(float black = 0.9) {
            SensorSet* sensor = &front_sensor;

            int start = millis();

            // Move while both left and right sensors detect white
            while (sensor->left->get_normalised() < black && sensor->right->get_normalised() < black) {
                move(153, 0.0);
            };

            int stopt = millis();
            move(-153, 0.0);
            delay(220);
            stop();
            align(false);
            move(-102, 0.0);
            delay(140);
            stop();
            return stopt - start;
        }

        /**
         * Move until a sensor detects black, just like check_front.
         * 
         * ARGS:
         *     float boost [DEFAULT=0.0f] : If non-zero, the robot would boost; starts fast, and slow down steadily.
         *     bool back_up [DEFAULT=true] : Apply opposite motor force to brake when this is set to true.
         *     bool backward [DEFAULT=false] : Whether move backward (true) or forward (false).
         *     int power [DEFAULT=178] : How much motor power you want to use.
         *     float black [DEFAULT=0.9] : The threshold where it is considered "Black" when the sensor value is less than this.
         * 
         * RETURNS:
         *     Nothing.
         */
        void run_until_black(float boost = 0.0f, bool back_up = true, bool backward = false, int pow = 178, float black = 0.9) {
            int DIR  = backward ? -1 : 1;
            SensorSet* sensor = backward ? &back_sensor : &front_sensor;
            
            int speed = pow;

            int now = millis();
            int start = now;
            int lastTime = now;
            
            int boostTime = 270.0f * fabs(boost);

            while (sensor->left->get_normalised() < black && sensor->right->get_normalised() < black) {
                now = millis();
                float dt = (now - lastTime) / 1000.0f;
                lastTime = now;
                if (dt <= 0) dt = 0.001f;
                
                if (boost > 0.0) {
                    speed = lerp(230.0, (double) pow, constrain((now - start) / boostTime, 0.0, 1.0));
                }

                double direction = 0.0;
                double ln = 0.0;
                double rn = 0.0;
                if (sensor_set_pair_controller->hasValue()) {
                    ln = backward ? sensor_set_pair_controller->getValue().right->get_normalised() : sensor_set_pair_controller->getValue().left->left->get_normalised();
                    rn = backward ? sensor_set_pair_controller->getValue().right->get_normalised() : sensor_set_pair_controller->getValue().right->left->get_normalised();    
                }

                if (!(ln < 0.9 && rn < 0.9)) {
                    double dir = ln - rn;
                    direction = constrain(dir, -1.0, 1.0);
                }
                
                move(DIR*speed, -direction);
            };

            move(DIR*-pow, 0.0);
            delay(220);
            stop();

            delay(100);
            align(backward);
            
            if (back_up) {
                move(DIR*-102, 0.0);
                delay(180);
                stop();
            }
        }
        
        /**
         * Move until the sensor detects white.
         * 
         * ARGS:
         *     bool backward [DEFAULT=false] : Whether move backward (true) or forward (false).
         * 
         * RETURNS:
         *     Nothing.
         */
        void run_until_white(bool backward = false) {
            int DIR  = backward ? -1 : 1;
            SensorSet* sensor = backward ? &back_sensor : &front_sensor;

            while (sensor->get_normalised() > 0.1) {
                double direction = 0.0;
                if (sensor_set_pair_controller->hasValue()) {
                    direction = sensor_set_pair_controller->getValue().get_direction();
                }
                if (fabs(direction) > 0.5) {
                    move(DIR*127, direction);
                } else {
                    move(DIR*127, 0.0);
                }
            };
        }
        
        /**
         * Move forward / backward to align with the black lines.
         * 
         * ARGS:
         *     bool backward [DEFAULT=true] : Whether move backward (true) or forward (false).
         * 
         * RETURNS:
         *     Nothing.
         */
        void align(bool backward = true) {
            SensorSet* nearSensor = backward ? &back_sensor : &front_sensor;
            int sensorCount = 1;
            SensorSet* sensors[2];
            if (far_back_sensor.hasValue()) {
                SensorSet* farSensor = backward ? &far_back_sensor.getValue() : &far_front_sensor.getValue();
                sensorCount = 2;
                sensors[0] = nearSensor;
                sensors[1] = farSensor;
            } else {
                sensors[0] = nearSensor;
            }


            int SEARCH_SPEED = backward ? -100 : 100;
            int ALIGN_DIR  = backward ? -1 : 1;
            const double BLACK_MIN = 0.5;    // bar detection threshold
            const double CENTER_EPS = 0.1;  // balance tolerance
        
            for (int i = 0; i<sensorCount; i++) {
                SensorSet* sensor = sensors[i];
                bool unable = false;

                int now = millis();
                int start = now;
                int lastTime = now;

                while (sensor->get_normalised() >= BLACK_MIN && !unable) {
                    if (now - start > 500) {
                        unable = true;
                        stop();
                        return;
                    }
                    now = millis();
                    move(-SEARCH_SPEED, 0.0);
                    unable = false;
                }
                while (sensor->get_normalised() < BLACK_MIN && !unable) {
                    if (now - start > 500) {
                        unable = true;
                        stop();
                        return;
                    }
                    now = millis();
                    move(SEARCH_SPEED, 0.0);
                    unable = false;
                }
                stop();

                now = millis();
                start = now;
                
                while (now - start < 2000 && !unable) {
                    now = millis();

                    float dt = (now - lastTime) / 1000.0f;
                    lastTime = now;
                    if (dt <= 0) dt = 0.001f;
                    
                    double strength = sensor->get_normalised();
                    double strength_error = 1.0 - strength;
                    
                    double dir = sensor->get_direction();
                    
                    if (fabs(dir) < CENTER_EPS) {
                        break;
                    }

                    int fb = (strength > BLACK_MIN) ? ALIGN_DIR * 140 : ALIGN_DIR * -140;
                
                    move(fb, dir);
                }
            }

            stop();
            resetIMUToLastPerfect();
        }
        
        /**
         * Gets the yaw of the robot.
         * 
         * ARGS:
         *     None.
         * 
         * RETURNS:
         *     float : The robot's current angle.
         */
        float getWorldYaw() {
            imu_sensor->getYaw();
            return norm180(worldYawOffset + imu_sensor->getYaw());
        }

        const float MAX_ERR = 90.0f;
        
        PID yawPID;
        
        /**
         * Rotate the robot to some direction.
         * 
         * ARGS:
         *     float absolute_degree : The target degree that the robot needs to rotate to.
         * 
         * RETURNS:
         *     Nothing.
         */
        void rotate_to(float absolute_degree) {
            float error = norm180(absolute_degree - getWorldYaw());
            float dir = (error > 0) ? 1.0f : -1.0f;
            
            if (fabs(error) >= 180) {
                yawPID = {3.6, 0.0, 0.7};
            } else {
                yawPID = {3.8, 0.0, 0.7}; //4.5, 0.0, 4.5
            }
            
            yawPID.reset();
            
            int lastTime = millis();
            float lastYaw = MAXFLOAT;
            int lastStallYawTime = lastTime;
            
            int minStallSpeed = 60;
            int maxStallSpeed = 120;
            float stallSpeed = minStallSpeed;
            int adjustSpeed = 40;
            int adjustSpeedTank = 0;

            int lastDir = dir;

            float yawDiff = 3.0;
            
            while (true) {
                clear();
                float yaw = getWorldYaw();
                int now = millis();
                float dt = (now - lastTime) / 1000.0f;
                lastTime = now;
                if (dt <= 0) dt = 0.001f;
                
                error = norm180(absolute_degree - yaw);
                float pidOut = yawPID.update(error, dt);
                dir = (error > 0) ? 1.0f : -1.0f;

                if (dir != lastDir) {
                    if (now - lastStallYawTime < 200) {
                        minStallSpeed = 5;
                        adjustSpeed = max(400-adjustSpeedTank, 50);
                        adjustSpeedTank += 60;
                    } else {
                        minStallSpeed = 60;
                        adjustSpeed = 60;
                    }

                    stallSpeed = minStallSpeed;
                    lastStallYawTime = now;
                }

                lastDir = dir;

                drawTextFmt(0,0,WHITE,"%f", stallSpeed);
                flip();

                if (fabs(yaw - lastYaw) > yawDiff) {
                    yawDiff = constrain(yawDiff-0.1, 0.05, 3.0);
                }
                lastYaw = yaw;

                stallSpeed = min(stallSpeed+(adjustSpeed*dt), (float) maxStallSpeed);
            
                if (fabs(error) < 0.035f) break;
                
                move(speedFromPID(pidOut, stallSpeed), dir);
            }
        
            stop();
        
            lastPerfectYaw = absolute_degree;
            resetIMUKeepWorld();
        }

        /**
         * Move forward while turning.
         *
         * ARGS:
         *     int absolute_degree : How much to turn in degrees.
         * 
         * RETURNS:
         *     Nothing.
         */
        void turnDegreeFront(int absolute_degree) {
            int min_speed = 153;   // ความเร็วต่ำสุด
            int max_speed = 200;   // ความเร็วสูงสุด

            float kp = 2.0;  // KP
            float kd = 1.0;  // KD

            float small_angle_threshold = 5;  // หุ่นยนต์จะใช้ความเร็วต่ำสุดเมื่อเข้าใกล้องศาที่กำหนด
            float stop_threshold = 1.0;       // กำหนดความคลาดเคลื่อนที่ยอมรับได้

            float previous_error = 0;

            while (1) {
                float error = getWorldYaw() - absolute_degree;

                int pd_value = abs((error * kp) + ((error - previous_error) * kd));

                if (pd_value > max_speed) pd_value = max_speed;
                if (pd_value < min_speed) pd_value = min_speed;
                    
                if (fabs(error) < stop_threshold) {
                    stop();
                    break;
                } else {
                    if (error <= 0) {
                        front.set(10, pd_value);
                        back.set(10, pd_value);
                    }
                    else if (error > 0) {
                        front.set(pd_value, 10);
                        back.set(pd_value, 10);
                    }
                }
                previous_error = error;
            }

            lastPerfectYaw = absolute_degree;
            resetIMUKeepWorld();
        }

    private:
        float worldYawOffset = 0.0f;
        float lastPerfectYaw = 0.0f;
        float continuousYaw = 0.0f;
        float lastYaw = 0.0f;
        bool yawInit = false;

        float unwrapYaw(float rawYaw) {
            if (!yawInit) {
                lastYaw = rawYaw;
                continuousYaw = rawYaw;
                yawInit = true;
                return continuousYaw;
            }
        
            float delta = rawYaw - lastYaw;
        
            // detect wrap (your IMU jumps ~370 deg)
            if (delta > 180)  delta -= 360;
            if (delta < -180) delta += 360;
        
            continuousYaw += delta;
            lastYaw = rawYaw;
            
            return continuousYaw;
        }
        
        /**
         * Calculate the clamped motor speed from PID output.
         * 
         * ARGS:
         *     float pidOut : The output of some PID to use.
         *     int stall_speed [DEFAULT=5] : The minimum speed.
         *     int speed_max [DEFAULT=250] : The maximum speed.
         * 
         * RETURNS:
         *     int : The clamped speed accounted for min / max speed.
         */
        int speedFromPID(float pidOut, int stall_speed = 5, int speed_max = 250) {
            float mag = fabs(pidOut);
            
            if (mag < stall_speed)
                mag = stall_speed;
        
                if (mag > speed_max)
                mag = speed_max;
                
            return (int)mag;
        }
        
        /**
         * Normalize an angle to range of -180 to 180.
         * NOTE: This method does not modify the original angle.
         * 
         * ARGS:
         *     float a : The input angle in degrees.
         * 
         * RETURNS:
         *     float : The normalized angle in range of -180 to 180.
         */
        float norm180(float a) {
            while (a > 180) a -= 360;
            while (a < -180) a += 360;
            return a;
        }

        void resetIMUKeepWorld() {
            imu_sensor->getYaw();
            worldYawOffset += imu_sensor->getYaw();
            imu_sensor->Reset();
        }

        void resetIMUToLastPerfect() {
            worldYawOffset = norm180(lastPerfectYaw);
            imu_sensor->Reset();
        }
    };