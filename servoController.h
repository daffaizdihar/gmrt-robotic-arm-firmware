#ifndef GMRT_SERVO_CONTROLLER_H
#define GMRT_SERVO_CONTROLLER_H

#include <Arduino.h>
#include <ESP32Servo.h>

constexpr uint8_t SERVO_COUNT = 5;

struct MotionConfig {
    uint32_t durationMs;
    uint16_t updateIntervalMs;
};

class ServoController {
public:
    bool begin();
    bool setReferencePosition(const int angles[SERVO_COUNT]);
    bool moveTo(const int target[SERVO_COUNT], uint32_t durationMs);
    void update();
    void stop();
    bool isBusy() const;
    bool hasReference() const;
    void getCurrentPosition(int out[SERVO_COUNT]) const;
    static const uint8_t *pins();

private:
    Servo servos[SERVO_COUNT];
    static const uint8_t servoPins[SERVO_COUNT];
    int current[SERVO_COUNT] = {90, 90, 90, 90, 90};
    int start[SERVO_COUNT] = {90, 90, 90, 90, 90};
    int target[SERVO_COUNT] = {90, 90, 90, 90, 90};
    bool attached[SERVO_COUNT] = {false, false, false, false, false};
    bool referenceValid = false;
    bool moving = false;
    uint32_t motionStartMs = 0;
    uint32_t motionDurationMs = 0;
    uint32_t lastUpdateMs = 0;
};

#endif
