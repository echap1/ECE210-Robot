//
// Created by Ethan Chapman on 5/10/23.
//

#include "MotorWrapper.h"
#include "Arduino.h"
#include "Motor.h"

void motor_init() {
    Motor_Init();
}

void motor_set(double r, double omega) {
    double v1 = (r - HALF_TRACK_WIDTH) * omega;
    double v2 = (r + HALF_TRACK_WIDTH) * omega;
    motor_set_vel(v1, v2);
}

void motor_set_vel(double v1, double v2) {
    int duty1 = (int) (100.0 * v1 / VEL_AT_100);
    int duty2 = (int) (100.0 * v2 / VEL_AT_100);
    digitalWrite(L_DIR, duty1 > 0);
    digitalWrite(R_DIR, duty2 > 0);
    analogWrite(L_PWM, constrain(abs(duty1), 0, 255));
    analogWrite(R_PWM, constrain(abs(duty2), 0, 255));
}