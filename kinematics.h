#ifndef GMRT_KINEMATICS_H
#define GMRT_KINEMATICS_H

struct ArmGeometry {
    float baseHeightCm;
    float shoulderToElbowCm; // L1
    float elbowToWristCm;    // L2
    float wristToToolTipCm;  // L3; measured along the distal arm axis
};

// Angles are mathematical joint angles in degrees.
// Wrist is modeled as roll about the distal arm axis, not pitch.
struct JointAngles {
    float base;
    float shoulder;
    float elbow;
    float wristRoll;
};

struct CartesianPose {
    float x;
    float y;
    float z;
    float wristRoll;
};

enum IKStatus {
    IK_SUCCESS,
    IK_INVALID_INPUT,
    IK_INVALID_GEOMETRY,
    IK_UNREACHABLE,
    IK_NUMERICAL_ERROR
};

void forwardKinematics(const ArmGeometry &geometry,
                       const JointAngles &joints,
                       CartesianPose &pose);

IKStatus inverseKinematics(const ArmGeometry &geometry,
                           const CartesianPose &target,
                           bool elbowPositive,
                           JointAngles &solution);

#endif
