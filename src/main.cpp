#ifndef ESP8266
#error This code is designed to run on ESP8266 platform! Please check your Tools->Board setting.
#endif

#define TIMER_INTERRUPT_DEBUG 0
#define ISR_SERVO_DEBUG 0

#include <PS2X_lib.h>

#include "Adafruit_PWMServoDriver.h"

#define SEC_IN_US 1000000
#define SERVO_MIN_US 500
#define SERVO_MAX_US 2500
#define SERVO_FREQ 50
#define NUM_SERVOS 1
uint16_t SERVO_MIN_PULSE = 4096.0 * SERVO_FREQ / SEC_IN_US * SERVO_MIN_US;
uint16_t SERVO_MAX_PULSE = 4096.0 * SERVO_FREQ / SEC_IN_US * SERVO_MAX_US;

Adafruit_PWMServoDriver servoDriver = Adafruit_PWMServoDriver();

struct ServoInfo {
    uint8_t id;
    int8_t dir;
    int16_t maxPos;
    int16_t minTgt;
    int16_t maxTgt;
};

ServoInfo servos[] = {{0, -1, 270, 0, 270}, {1, 1, 270, 0, 180},
                      {2, -1, 270, 0, 270}, {3, 1, 180, 0, 180},
                      {4, 1, 180, 0, 180},  {5, 1, 180, 120, 180}};

void setServoPos(ServoInfo servo, int16_t pos) {
    if (pos < servo.minTgt) {
        pos = servo.minTgt;
    } else if (pos > servo.maxTgt) {
        pos = servo.maxTgt;
    }
    servoDriver.setPWM(
        servo.id, 0,
        map(pos, 0, servo.maxPos, SERVO_MIN_PULSE, SERVO_MAX_PULSE));
}

int16_t getServoPos(ServoInfo servo) {
    uint16_t offPulse = servoDriver.getPWM(servo.id, true);
    return map(offPulse, SERVO_MIN_PULSE, SERVO_MAX_PULSE, 0, servo.maxPos);
}

void resetServos() {
    Serial.printf("reset servos\n");
    setServoPos(servos[0], 135);
    delay(150);
    setServoPos(servos[1], 0);
    delay(200);
    setServoPos(servos[2], 0);
    delay(250);
    setServoPos(servos[3], 0);
    delay(300);
    setServoPos(servos[4], 0);
    delay(400);
    setServoPos(servos[5], 180);
    delay(500);
}
void setupServos() {
    Serial.printf("servo min pulse=%d, max pulse=%d\n", SERVO_MIN_PULSE,
                  SERVO_MAX_PULSE);
    servoDriver.begin();
    // servoDriver.setOscillatorFrequency(25000000);
    servoDriver.setPWMFreq(SERVO_FREQ);
    delay(10);
    resetServos();
}

int16_t moveServo(ServoInfo servo, int16_t delta) {
    if (delta == 0) {
        return 0;
    }
    int16_t curPos = getServoPos(servo);
    int16_t tgtPos =
        max(servo.minTgt, min(servo.maxTgt, (int16_t)(curPos + delta)));
    Serial.printf("servo=%d, delta=%d, curPos=%d, tgtPos=%d\n", servo.id, delta,
                  curPos, tgtPos);
    if (tgtPos != curPos) {
        setServoPos(servo, tgtPos);
    }
    return tgtPos - curPos;
}

int16_t moveServoByStick(ServoInfo servo, byte stickPos) {
    int16_t delta = stickPos >= 91 && stickPos <= 164 ? 0
                    : stickPos > 164                  ? 1
                                                      : -1;
    return moveServo(servo, delta * servo.dir);
}

int16_t moveClaw(ServoInfo servo, int8_t dir) {
    if (dir == 0) {
        return 0;
    }
    int16_t delta = dir > 0 ? 1 : -1;
    return moveServo(servo, delta);
}

PS2X ps2x;
struct PS2X_Info_t {
    bool pressures = true;
    bool rumble = false;
    int error = 1;
    byte type = 0;
} ps2xInfo;

void setupPS2X() {
    ps2xInfo.error = ps2x.config_gamepad(D3, D4, D5, D6, ps2xInfo.pressures,
                                         ps2xInfo.rumble);
    if (ps2xInfo.error == 1) {
        Serial.println("No controller found.");
    } else if (ps2xInfo.error == 2) {
        Serial.println("Controller found but not accepting commands.");
    } else if (ps2xInfo.error == 3) {
        Serial.println(
            "Controller refusing to enter Pressures mode, may not support it.");
    }

    ps2xInfo.type = ps2x.readType();
    switch (ps2xInfo.type) {
        case 0:
            Serial.println("Unknown Controller type found ");
            break;
        case 1:
            Serial.println("DualShock Controller found ");
            break;
        case 2:
            Serial.println("GuitarHero Controller found ");
            break;
        case 3:
            Serial.println("Wireless Sony DualShock Controller found ");
            break;
    }
}

void reactPS2X() {
    if (ps2xInfo.error == 1) {
        delay(50);
        return;
    }
    ps2x.read_gamepad(false, false);
    if (ps2x.Button(PSB_START)) {
        resetServos();
        return;
    }

    byte ly = ps2x.Analog(PSS_LY);
    byte lx = ps2x.Analog(PSS_LX);
    byte ry = ps2x.Analog(PSS_RY);
    byte rx = ps2x.Analog(PSS_RX);
    // Serial.printf("ly=%d, lx=%d, ry=%d, rx=%d\n", ly, lx, ry, rx);

    moveServoByStick(servos[0], lx);
    int16_t moved = moveServoByStick(servos[1], ly);
    if (moved) {
        moveServo(servos[3], -moved);
    }
    if (ps2x.Button(PSB_R1)) {
        moveServoByStick(servos[3], ry);
    } else {
        moved = moveServoByStick(servos[2], ry);
        if (moved) {
            moveServo(servos[3], moved);
        }
    }
    moveServoByStick(servos[4], rx);
    if (ps2x.Button(PSB_CIRCLE)) {
        moveClaw(servos[5], -1);
    } else if (ps2x.Button(PSB_CROSS)) {
        moveClaw(servos[5], 1);
    }
    delay(5);
}

void setup() {
    Serial.begin(9600);
    Serial.printf("\nStarting\n");
    setupServos();
    setupPS2X();
}

void loop() {
    // react
    reactPS2X();
}