//
// Created by Ethan Chapman on 5/10/23.
//

#include "WallFollowing.h"
#include "Arduino.h"
#include "SharpDistSensor.h"
#include "PIDController.h"
#include "MotorWrapper.h"

#define OFFSET 50       // dist from IR sensor to center of robot in mm
#define DESIRED 100     // desired dist from wall
#define PWMNOMINAL 100  // nominal speed
#define SWING 20        // limits on speed
#define PWMMIN (PWMNOMINAL-SWING)
#define PWMMAX (PWMNOMINAL+SWING)

namespace wall_following {
    const byte irR = A2;
    const byte irC = A1;
    const byte irL = A0;

    const byte medianFilterWindowSize = 5;

    SharpDistSensor sensorR(irR, medianFilterWindowSize);
    SharpDistSensor sensorC(irC, medianFilterWindowSize);
    SharpDistSensor sensorL(irL, medianFilterWindowSize);

    PIDController pid(0.005, 0, 0.1, 100, -1, 1);

    void setup() {
        sensorR.setModel(SharpDistSensor::GP2Y0A51SK0F_5V_DS);
        sensorC.setModel(SharpDistSensor::GP2Y0A51SK0F_5V_DS);
        sensorL.setModel(SharpDistSensor::GP2Y0A51SK0F_5V_DS);
    }

    double t_last = 0;

    void loop() {
        Serial.println("#############");
        unsigned int distR = sensorR.getDist() + OFFSET;
        Serial.print("Right: ");
        Serial.println(distR);
        unsigned int distC = sensorC.getDist() + OFFSET;
        Serial.print("Center: ");
        Serial.println(distC);
        unsigned int distL = sensorL.getDist() + OFFSET;
        Serial.print("Left: ");
        Serial.println(distL);

        double diff = (double) distL - (double) distR;


        double fw_vel = 0.35;
        double pid_factor = 0.3;

//        double error = diff;
        double error = (double) distL - 100.0;

        if (distC < 120) {
            error = -(double) distC;
            fw_vel *= 0.2;
        }

        double t = millis();
        double pid_out = pid.calculate(0, error, t, t - t_last);
        t_last = t;

        motor_set_vel(fw_vel + pid_factor * pid_out, fw_vel - pid_factor * pid_out);
    }
}