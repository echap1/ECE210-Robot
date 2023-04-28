//
// Created by Ethan Chapman on 4/18/23.
//

#ifndef ECE210_ROBOT_PIDCONTROLLER_H
#define ECE210_ROBOT_PIDCONTROLLER_H

#include "Arduino.h"

#define INTEGRAL_SIZE 1000


class PIDController {
public:
    PIDController(double kp, double ki, double kd, double measureWindow, double min_output, double max_output);

    double calculate(double setpoint, double measurement, double t, double dt);

private:
    double kp_;
    double ki_;
    double kd_;
    double min_output_;
    double max_output_;
    double integral_;
    double integralMeasurements[INTEGRAL_SIZE][2];  // Time, Measurement
    int measureStart;
    int measureStop;
    double measureWindow;
    double prev_error_;
};


#endif //ECE210_ROBOT_PIDCONTROLLER_H
