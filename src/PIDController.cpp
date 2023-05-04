//
// Created by Ethan Chapman on 4/18/23.
//

#include "PIDController.h"

PIDController::PIDController(double kp, double ki, double kd, double measureWindow, double min_output, double max_output) :
        kp_(kp), ki_(ki), kd_(kd), min_output_(min_output), max_output_(max_output),
        integral_(0), measureStart(0), measureStop(0), measureWindow(measureWindow), prev_error_(0) {}

double PIDController::calculate(double setpoint, double measurement, double t, double dt) {
    double error = setpoint - measurement;
    integral_ = 0;

    int i = measureStart;
    double cutoff = t - measureWindow;
    for (;; i = (i + 1) % INTEGRAL_SIZE) {
        if (integralMeasurements[i][0] >= cutoff) {
            measureStart = i;
            break;
        }
        if (i == measureStop) break;
    }
    measureStop = (measureStop + 1) % INTEGRAL_SIZE;
    integralMeasurements[measureStop][0] = t;
    integralMeasurements[measureStop][1] = error * dt / 1000;
    for (;; i = (i + 1) % INTEGRAL_SIZE) {
        integral_ += integralMeasurements[i][1];
        if (i == measureStop) break;
    }
    if (measureStart == measureStop) {
        measureStart = (measureStart + 1) % INTEGRAL_SIZE;
    }

    double derivative = (error - prev_error_) / dt;
    prev_error_ = error;

    Serial.print("derivative: "); Serial.println(kd_ * derivative);
    Serial.print("error: "); Serial.println(kp_ * error);

    double output = kp_ * error + ki_ * integral_ + kd_ * derivative;
    return max(min(output, max_output_), min_output_);
}
