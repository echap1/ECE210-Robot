/* 
 *  Filename: PCB_test.ino
 *  Author: Capt Steven Beyer
 *  Created: 6 Mar 2019
 *  Description: Arduino Sketch that tests the operation of the PCB board.
 *    If the board is working correctly both motors will drive forward for
 *    one second and then the right motor will go backwards while the left
 *    motor will go forwards (right turn). Both motors will stop and pause
 *    for one second.
 *  
 *  Required Files:
 *    Libraries : none
 *    Packages  : none
 *    Files     : Motor.h
 */

#include "Arduino.h"
#include "Motor.h"
#include "QTR-8RC.h"
#include "PIDController.h"

#define WHEEL_DIAMETER   0.06985  // meters
#define TRACK_WIDTH      0.09525  // meters
#define HALF_TRACK_WIDTH 0.047625 // meters

#define VEL_AT_100     0.21     // m/s

#define HISTORY_LEN 10
#define THRESHOLD 5
int16_t linePositions[HISTORY_LEN] = {-1, -1, -1, -1, -1, -1, -1, -1, -1, -1};

QTRSensors qtr;
const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];

#define kP 0.17
#define kI 0  //0.0001
#define kD (-250 * 1000)
PIDController pid(kP, kI, kD, 1000, -1, 1);

void motor_set(double r, double omega);
void motor_set_vel(double v1, double v2);

double t_last = 0;

void setup() {
    Serial.begin(115200);
    Serial.println("Initializing...");

    // initialize robot instance setting pins and turn delay
    Motor_Init();

    qtr.setTypeRC();
    qtr.setSensorPins((const uint8_t[]){6, 7, 8, 9, 10, 11, 12, 13}, SensorCount);
    //qtr.setEmitterPin(5);

    delay(500);
    Serial.println("Calibrating...");

    // During calibration
    // 2.5 ms RC read timeout (default) * 10 reads per calibrate() call
    // = ~25 ms per calibrate() call.
    // Call calibrate() 400 times to make calibration take about 10 seconds.
    for (uint16_t i = 0; i < 400; i++) {
        qtr.calibrate();
    }

    Serial.print("Calibration Done!\nMaximum Values: ");
    for (uint8_t i = 0; i < SensorCount; i++) {
        Serial.print(qtr.calibrationOn.maximum[i]);
        Serial.print(' ');
    }
    Serial.print("\nMinimum Values: ");
    for (uint8_t i = 0; i < SensorCount; i++) {
        Serial.print(qtr.calibrationOn.minimum[i]);
        Serial.print(' ');
    }

    delay(1000);

    t_last = millis();
}

double radius = 0.5;
double dr = -0.002;

double last_valid_pos = 0;
int valid_pos_len = 0;

// basic code to go forward for a second and then turn right
void loop() {
    // MEASURE
    int16_t pos = qtr.readLineWhite(sensorValues);
    byte valid_count = 0;
    double avg_pos = 0;
    bool on_line;
    for (byte i = 1; i < HISTORY_LEN; i++) {
        // Shift left
        linePositions[i - 1] = linePositions[i];
        if (linePositions[i] != -1) {
            valid_count++;
            avg_pos += (double) linePositions[i];
        }
    }
    linePositions[HISTORY_LEN - 1] = pos;  // Add new pos to the end
    if (pos != -1) {
        valid_count++;
        avg_pos += (double) pos;
    }

    on_line = valid_count >= THRESHOLD;
    avg_pos /= (double) valid_count;
    avg_pos /= 10000;  // To meters

//    for (short linePosition : linePositions) {
//        Serial.print(linePosition);
//        Serial.print(" ");
//    }
//    Serial.println();

    double pid_out = 0;

    if (on_line) {
        last_valid_pos = avg_pos;
        valid_pos_len = 1000;
    } else if (valid_pos_len > 0) {
        avg_pos = last_valid_pos * 1.1;
    } else {
        Serial.println("Not on line!");
        motor_set_vel(0, 0);
        return;
    }

    Serial.println(avg_pos);

    double error = avg_pos / 0.0332;
    double t = millis();
    Serial.print("DT: ");
    Serial.println(t - t_last);
    pid_out = pid.calculate(0, error, t - t_last, t);
    Serial.print("PID OUT: ");
    Serial.println(pid_out);
    t_last = t;
//  double fw_vel = 0.1 * (2 - abs(error));
    double fw_vel = on_line ? 0.1 : 0.075;
    motor_set_vel(fw_vel + 0.15 * pid_out, fw_vel - 0.15 * pid_out);

//    } else {
//        Serial.println("Not on line!");
//        motor_set_vel(0, 0);
//    }

//    delay(100);
}

void motor_set(double r, double omega) {
    double v1 = (r - HALF_TRACK_WIDTH) * omega;
    double v2 = (r + HALF_TRACK_WIDTH) * omega;
    motor_set_vel(v1, v2);
}

void motor_set_vel(double v1, double v2) {
    int duty1 = (int) (100.0 * v1 / VEL_AT_100);
    int duty2 = (int) (100.0 * v2 / VEL_AT_100);
    digitalWrite(L_DIR,duty1 > 0);
    digitalWrite(R_DIR,duty2 > 0);
    analogWrite(L_PWM, constrain(abs(duty1), 0, 255));
    analogWrite(R_PWM, constrain(abs(duty2), 0, 255));
}