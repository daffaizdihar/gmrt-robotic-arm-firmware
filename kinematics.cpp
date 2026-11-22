#include "kinematics.h"
#include <math.h>

namespace {
constexpr float PI_F = 3.14159265358979323846f;
constexpr float DEG_TO_RAD_F = PI_F / 180.0f;
constexpr float RAD_TO_DEG_F = 180.0f / PI_F;
constexpr float EPSILON = 1.0e-5f;

bool validGeometry(const ArmGeometry &g) {
    return isfinite(g.baseHeightCm) &&
           isfinite(g.shoulderToElbowCm) &&
           isfinite(g.elbowToWristCm) &&
           isfinite(g.wristToToolTipCm) &&
           g.baseHeightCm >= 0.0f &&
           g.shoulderToElbowCm > 0.0f &&
           g.elbowToWristCm > 0.0f &&
           g.wristToToolTipCm >= 0.0f;
}
}

void forwardKinematics(const ArmGeometry &g,
                       const JointAngles &j,
                       CartesianPose &p) {
    const float base = j.base * DEG_TO_RAD_F;
    const float shoulder = j.shoulder * DEG_TO_RAD_F;
    const float elbow = j.elbow * DEG_TO_RAD_F;

    // Assumption from the mechanical description:
    // J4 rotates the gripper around the distal link axis (roll).
    // Therefore J4 changes tool orientation, not the XYZ tip position.
    // L2 and L3 lie along the same distal pitch direction.
    const float distalLength = g.elbowToWristCm + g.wristToToolTipCm;
    const float distalPitch = shoulder + elbow;

    const float radial = g.shoulderToElbowCm * cosf(shoulder) +
                         distalLength * cosf(distalPitch);

    p.x = radial * cosf(base);
    p.y = radial * sinf(base);
    p.z = g.baseHeightCm +
          g.shoulderToElbowCm * sinf(shoulder) +
          distalLength * sinf(distalPitch);
    p.wristRoll = j.wristRoll;
}

IKStatus inverseKinematics(const ArmGeometry &g,
                           const CartesianPose &target,
                           bool elbowPositive,
                           JointAngles &solution) {
    if (!isfinite(target.x) || !isfinite(target.y) ||
        !isfinite(target.z) || !isfinite(target.wristRoll)) {
        return IK_INVALID_INPUT;
    }
    if (!validGeometry(g)) return IK_INVALID_GEOMETRY;

    const float rho = hypotf(target.x, target.y);
    const float base = (rho > EPSILON) ? atan2f(target.y, target.x) : 0.0f;
    const float z = target.z - g.baseHeightCm;
    const float distal = g.elbowToWristCm + g.wristToToolTipCm;

    const float d2 = rho * rho + z * z;
    const float l1 = g.shoulderToElbowCm;
    const float l2 = distal;
    float cElbow = (d2 - l1 * l1 - l2 * l2) / (2.0f * l1 * l2);

    if (!isfinite(cElbow)) return IK_NUMERICAL_ERROR;
    if (cElbow < -1.0f - EPSILON || cElbow > 1.0f + EPSILON)
        return IK_UNREACHABLE;

    cElbow = fmaxf(-1.0f, fminf(1.0f, cElbow));
    float sElbow = sqrtf(fmaxf(0.0f, 1.0f - cElbow * cElbow));
    if (!elbowPositive) sElbow = -sElbow;

    const float elbow = atan2f(sElbow, cElbow);
    const float shoulder = atan2f(z, rho) -
        atan2f(l2 * sinf(elbow), l1 + l2 * cosf(elbow));

    solution.base = base * RAD_TO_DEG_F;
    solution.shoulder = shoulder * RAD_TO_DEG_F;
    solution.elbow = elbow * RAD_TO_DEG_F;
    solution.wristRoll = target.wristRoll;

    if (!isfinite(solution.base) || !isfinite(solution.shoulder) ||
        !isfinite(solution.elbow) || !isfinite(solution.wristRoll))
        return IK_NUMERICAL_ERROR;

    return IK_SUCCESS;
}
