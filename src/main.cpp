#include "Arduino.h"
#include "MotorWrapper.h"
#include "LineFollowing.h"
#include "WallFollowing.h"

void setup() {
    motor_init();
    Serial.begin(115200);

//    line_following_setup();
    wall_following::setup();
}

void loop() {
//    line_following_loop();
    wall_following::loop();
}
