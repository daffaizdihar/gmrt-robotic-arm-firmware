#include "servoController.h"
#include <math.h>

const uint8_t ServoController::servoPins[SERVO_COUNT] = {13, 14, 27, 26, 25};

bool ServoController::begin() {
    bool ok = true;
    for (uint8_t i = 0; i < SERVO_COUNT; ++i) {
        servos[i].setPeriodHertz(50);
        const int channel = servos[i].attach(servoPins[i], 500, 2400);
        attached[i] = (channel >= 0 && servos[i].attached());
        if (!attached[i]) ok = false;
        // Deliberately do not call write(90) here. The physical pose is unknown.
    }
    return ok;
}

bool ServoController::setReferencePosition(const int angles[SERVO_COUNT]) {
    if (!angles) return false;
    for (uint8_t i = 0; i < SERVO_COUNT; ++i) {
        if (!attached[i] || angles[i] < 0 || angles[i] > 180) return false;
    }
    for (uint8_t i = 0; i < SERVO_COUNT; ++i) {
        current[i] = start[i] = target[i] = angles[i];
    }
    referenceValid = true;
    moving = false;
    return true;
}

bool ServoController::moveTo(const int requested[SERVO_COUNT], uint32_t durationMs) {
    if (!requested || !referenceValid || moving) return false;
    for (uint8_t i = 0; i < SERVO_COUNT; ++i) {
        if (!attached[i] || requested[i] < 0 || requested[i] > 180) return false;
    }
    for (uint8_t i = 0; i < SERVO_COUNT; ++i) {
        start[i] = current[i];
        target[i] = requested[i];
    }
    motionStartMs = millis();
    lastUpdateMs = motionStartMs;
    motionDurationMs = (durationMs == 0) ? 1 : durationMs;
    moving = true;
    return true;
}

void ServoController::update() {
    if (!moving) return;
    const uint32_t now = millis();
    if (now - lastUpdateMs < 20U) return;
    lastUpdateMs = now;

    float t = static_cast<float>(now - motionStartMs) /
              static_cast<float>(motionDurationMs);
    if (t >= 1.0f) t = 1.0f;
    // Smoothstep: synchronized interpolation with gentle start/end.
    const float eased = t * t * (3.0f - 2.0f * t);

    for (uint8_t i = 0; i < SERVO_COUNT; ++i) {
        const float value = static_cast<float>(start[i]) +
            (static_cast<float>(target[i] - start[i]) * eased);
        const int command = static_cast<int>(lroundf(value));
        if (command != current[i]) {
            servos[i].write(command);
            current[i] = command;
        }
    }

    if (t >= 1.0f) moving = false;
}

void ServoController::stop() {
    // Freeze at the last commanded positions (open-loop; not measured positions).
    moving = false;
    for (uint8_t i = 0; i < SERVO_COUNT; ++i) {
        start[i] = target[i] = current[i];
    }
}

bool ServoController::isBusy() const { return moving; }
bool ServoController::hasReference() const { return referenceValid; }

void ServoController::getCurrentPosition(int out[SERVO_COUNT]) const {
    if (!out) return;
    for (uint8_t i = 0; i < SERVO_COUNT; ++i) out[i] = current[i];
}

const uint8_t *ServoController::pins() { return servoPins; }
