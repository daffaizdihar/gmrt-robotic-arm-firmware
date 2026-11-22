#include <Arduino.h>
#include <math.h>
#include <ctype.h>
#include <stdlib.h>
#include <string.h>

#include "kinematics.h"
#include "servoController.h"

// ==================== HARDWARE / GEOMETRY ====================
// Mechanical data received 22 Nov 2026. Units: cm.
// BASE_HEIGHT = base rotation axis to shoulder rotation axis.
// L3 = wrist rotation axis to gripper contact tip.
// J4 is reported as wrist rotation (roll), NOT pitch.
static const ArmGeometry ARM = {8.0f, 13.5f, 7.0f, 7.5f};
static const uint32_t DEFAULT_MOVE_MS = 1800;
static const uint32_t MIN_MOVE_MS = 100;
static const uint32_t MAX_MOVE_MS = 15000;
static const size_t INPUT_SIZE = 128;

// PROVISIONAL calibration only. Direction/offset must be physically calibrated.
// Servo command = offset + direction * mathematical joint angle.
struct ServoCalibration { float offset; int8_t direction; float minCommand; float maxCommand; };
static const ServoCalibration CAL[4] = {
    {90.0f, +1, 0.0f, 180.0f}, // J1 Base
    {90.0f, +1, 0.0f, 180.0f}, // J2 Shoulder
    {90.0f, +1, 0.0f, 180.0f}, // J3 Elbow
    {90.0f, +1, 0.0f, 180.0f}  // J4 Wrist roll
};

ServoController armController;
char inputBuffer[INPUT_SIZE];
size_t inputIndex = 0;
bool inputOverflow = false;
uint32_t moveDurationMs = DEFAULT_MOVE_MS;

static bool finiteServo(float v) { return isfinite(v) && v >= 0.0f && v <= 180.0f; }
static float servoFromJoint(float joint, const ServoCalibration &c) { return c.offset + c.direction * joint; }
static float jointFromServo(float servo, const ServoCalibration &c) { return c.direction * (servo - c.offset); }

static void printHelp() {
    Serial.println(F("\n=== GMRT ROBOT ARM / ESP32 ==="));
    Serial.println(F("H                         : help"));
    Serial.println(F("STATUS                    : status"));
    Serial.println(F("TIME,milliseconds         : set motion duration (100..15000)"));
    Serial.println(F("SYNC,b,s,e,r,g            : adopt CURRENT physical servo positions; no move"));
    Serial.println(F("J,b,s,e,r,g               : direct servo command angles (0..180)"));
    Serial.println(F("IK,x,y,z,roll,gripper     : Cartesian XYZ + wrist roll + gripper servo angle"));
    Serial.println(F("IKU,...                   : force positive elbow branch"));
    Serial.println(F("IKD,...                   : force negative elbow branch"));
    Serial.println(F("STOP                      : stop interpolation"));
    Serial.println(F("Units: XYZ cm; angles degrees; J/Gripper values are servo angles."));
    Serial.println(F("NOTE: wrist J4 is modeled as roll about distal link axis."));
}

static void printStatus() {
    int p[SERVO_COUNT];
    armController.getCurrentPosition(p);
    Serial.printf("Attached: %s | Reference: %s | Moving: %s | Duration: %lu ms\n",
        "see boot log", armController.hasReference() ? "YES" : "NO",
        armController.isBusy() ? "YES" : "NO", (unsigned long)moveDurationMs);
    Serial.printf("Commanded positions: %d,%d,%d,%d,%d\n", p[0],p[1],p[2],p[3],p[4]);
    Serial.println(F("GPIO: J1=13 J2=14 J3=27 J4=26 J5=25"));
}

static bool parseCsv(char *line, float *v, size_t count) {
    char *p = strchr(line, ',');
    if (!p) return false;
    ++p;
    for (size_t i=0; i<count; ++i) {
        while (isspace((unsigned char)*p)) ++p;
        if (*p == '\0') return false;
        char *end = nullptr;
        v[i] = strtof(p, &end);
        if (end == p || !isfinite(v[i])) return false;
        while (isspace((unsigned char)*end)) ++end;
        if (i+1<count) {
            if (*end != ',') return false;
            p = end+1;
        } else {
            if (*end != '\0') return false;
        }
    }
    return true;
}

static bool validateServoTargets(const float f[5], int out[5]) {
    for (int i=0; i<5; ++i) {
        if (!finiteServo(f[i])) return false;
        out[i] = (int)lroundf(f[i]);
    }
    for (int i=0; i<4; ++i) {
        if (f[i] < CAL[i].minCommand || f[i] > CAL[i].maxCommand) return false;
    }
    return true;
}

static void printFK(const JointAngles &j) {
    CartesianPose p;
    forwardKinematics(ARM, j, p);
    Serial.printf("FK XYZ = (%.3f, %.3f, %.3f) cm | Wrist roll = %.2f deg\n",
                  p.x,p.y,p.z,p.wristRoll);
}

static void processJoint(const float v[5]) {
    if (armController.isBusy()) { Serial.println(F("[BUSY] Wait for current move or send STOP.")); return; }
    int target[5];
    if (!validateServoTargets(v,target)) { Serial.println(F("[REJECTED] Servo command outside configured 0..180 range.")); return; }
    if (!armController.hasReference()) { Serial.println(F("[REJECTED] Run SYNC first after physically aligning the arm.")); return; }
    if (!armController.moveTo(target,moveDurationMs)) { Serial.println(F("[ERROR] Move could not be started.")); return; }
    JointAngles j = {jointFromServo(v[0],CAL[0]),jointFromServo(v[1],CAL[1]),
                     jointFromServo(v[2],CAL[2]),jointFromServo(v[3],CAL[3])};
    printFK(j);
    Serial.println(F("[OK] Joint interpolation started."));
}

static void processIK(const float v[5], int branch) {
    if (armController.isBusy()) { Serial.println(F("[BUSY] Wait for current move or send STOP.")); return; }
    if (!armController.hasReference()) { Serial.println(F("[REJECTED] Run SYNC first after physically aligning the arm.")); return; }
    if (!finiteServo(v[3]) || !finiteServo(v[4])) { Serial.println(F("[REJECTED] Wrist-roll and gripper servo angles must be 0..180.")); return; }

    CartesianPose target = {v[0],v[1],v[2],jointFromServo(v[3],CAL[3])};
    JointAngles candidates[2];
    const IKStatus status[2] = {
        inverseKinematics(ARM,target,true,candidates[0]),
        inverseKinematics(ARM,target,false,candidates[1])
    };
    bool valid[2] = {false,false};
    float servoFloat[2][5] = {};
    for (int b=0;b<2;++b) {
        if (status[b] != IK_SUCCESS) continue;
        const float q[4] = {candidates[b].base,candidates[b].shoulder,candidates[b].elbow,candidates[b].wristRoll};
        valid[b] = true;
        for (int i=0;i<4;++i) {
            servoFloat[b][i] = servoFromJoint(q[i],CAL[i]);
            if (!isfinite(servoFloat[b][i]) || servoFloat[b][i] < CAL[i].minCommand || servoFloat[b][i] > CAL[i].maxCommand) valid[b] = false;
        }
        servoFloat[b][4] = v[4];
    }

    int selected = -1;
    if (branch > 0) selected = valid[0] ? 0 : -1;
    else if (branch < 0) selected = valid[1] ? 1 : -1;
    else if (valid[0]) selected = 0;
    else if (valid[1]) selected = 1;

    if (selected < 0) {
        if (status[0] == IK_UNREACHABLE && status[1] == IK_UNREACHABLE)
            Serial.println(F("[IK REJECTED] Target outside geometric workspace."));
        else Serial.println(F("[IK REJECTED] No branch satisfies geometric and provisional servo limits."));
        return;
    }

    int targetServo[5];
    if (!validateServoTargets(servoFloat[selected],targetServo)) { Serial.println(F("[IK REJECTED] Servo target invalid.")); return; }
    if (!armController.moveTo(targetServo,moveDurationMs)) { Serial.println(F("[ERROR] Move could not be started.")); return; }
    Serial.printf("[IK] Branch=%s | J1=%.2f J2=%.2f J3=%.2f J4-roll=%.2f deg\n",
        selected==0 ? "positive" : "negative", candidates[selected].base,
        candidates[selected].shoulder,candidates[selected].elbow,candidates[selected].wristRoll);
    Serial.println(F("[OK] Cartesian interpolation started. XYZ is modeled with wrist-roll axis collinear to distal link."));
}

static void processCommand(char *line) {
    while (isspace((unsigned char)*line)) ++line;
    size_t n = strlen(line);
    while (n && isspace((unsigned char)line[n-1])) line[--n]='\0';
    if (!n) return;
    if (!strcmp(line,"H") || !strcmp(line,"h")) { printHelp(); return; }
    if (!strcmp(line,"STATUS")) { printStatus(); return; }
    if (!strcmp(line,"STOP")) { armController.stop(); Serial.println(F("[STOP] Interpolation stopped at last commanded pose.")); return; }

    if (!strncmp(line,"TIME,",5)) {
        float v[1];
        if (!parseCsv(line,v,1) || v[0] < MIN_MOVE_MS || v[0] > MAX_MOVE_MS) {
            Serial.println(F("[ERROR] TIME must be 100..15000 ms.")); return;
        }
        moveDurationMs = (uint32_t)lroundf(v[0]);
        Serial.printf("[OK] Motion duration = %lu ms\n",(unsigned long)moveDurationMs); return;
    }
    if (!strncmp(line,"SYNC,",5)) {
        if (armController.isBusy()) { Serial.println(F("[BUSY] Send STOP before SYNC.")); return; }
        float f[5]; int p[5];
        if (!parseCsv(line,f,5) || !validateServoTargets(f,p)) { Serial.println(F("[ERROR] SYNC format/range invalid.")); return; }
        if (armController.setReferencePosition(p)) Serial.println(F("[SYNC] Reference stored; no servo movement commanded. Ensure the physical arm was manually aligned to these angles."));
        else Serial.println(F("[ERROR] SYNC failed; verify all servo attachments."));
        return;
    }
    if (!strncmp(line,"J,",2)) {
        float v[5]; if (!parseCsv(line,v,5)) { Serial.println(F("[ERROR] Use J,b,s,e,r,g.")); return; }
        processJoint(v); return;
    }
    int branch = 0; size_t prefix = 0;
    if (!strncmp(line,"IKU,",4)) { branch=1; prefix=4; }
    else if (!strncmp(line,"IKD,",4)) { branch=-1; prefix=4; }
    else if (!strncmp(line,"IK,",3)) { branch=0; prefix=3; }
    if (prefix) {
        (void)prefix;
        float v[5]; if (!parseCsv(line,v,5)) { Serial.println(F("[ERROR] Use IK,x,y,z,wristRollServoAngle,gripperServoAngle.")); return; }
        processIK(v,branch); return;
    }
    Serial.println(F("[ERROR] Unknown command. Type H."));
}

void setup() {
    Serial.begin(115200);
    delay(300);
    Serial.println(F("\nGMRT ROBOT ARM - Day 4 hardware firmware"));
    Serial.println(F("Geometry: BASE_HEIGHT=8.0, L1=13.5, L2=7.0, L3=7.5 cm"));
    Serial.println(F("GPIO: J1=13 J2=14 J3=27 J4=26 J5=25"));
    Serial.println(F("J4 model: wrist roll; physical axis assumption must be confirmed."));
    const bool ok = armController.begin();
    Serial.println(ok ? F("[OK] Servo attachment completed.") : F("[WARN] One or more servo attachments failed."));
    Serial.println(F("[SAFETY] No startup position command is sent. Remove horns/linkages for first attach test."));
    printHelp();
}

void loop() {
    armController.update();
    while (Serial.available() > 0) {
        const char c = (char)Serial.read();
        if (c == '\r') continue;
        if (c == '\n') {
            if (inputOverflow) Serial.println(F("[ERROR] Input line too long."));
            else { inputBuffer[inputIndex]='\0'; processCommand(inputBuffer); }
            inputIndex=0; inputOverflow=false; continue;
        }
        if (!inputOverflow) {
            if (inputIndex < INPUT_SIZE-1) inputBuffer[inputIndex++]=c;
            else inputOverflow=true;
        }
    }
}
