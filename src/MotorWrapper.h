//
// Created by Ethan Chapman on 5/10/23.
//

#ifndef ECE210_ROBOT_MOTORWRAPPER_H
#define ECE210_ROBOT_MOTORWRAPPER_H

#define WHEEL_DIAMETER   0.06985  // meters
#define TRACK_WIDTH      0.09525  // meters
#define HALF_TRACK_WIDTH 0.047625 // meters

#define VEL_AT_100     0.21     // m/s

void motor_set(double r, double omega);
void motor_set_vel(double v1, double v2);

#endif //ECE210_ROBOT_MOTORWRAPPER_H
