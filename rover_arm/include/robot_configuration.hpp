#ifndef __ROBOT_CONFIGURATION__
#define __ROBOT_CONFIGURATION__

constexpr float MAX_JOINT_SPEED = 0.0125;
constexpr float MAX_LIN_SPEED = 0.0125f;

constexpr float MAX_VELOCITY_JL = MAX_LIN_SPEED;
constexpr float MAX_VELOCITY_J0 = MAX_JOINT_SPEED;
constexpr float MAX_VELOCITY_J1 = MAX_JOINT_SPEED;
constexpr float MAX_VELOCITY_J2 = MAX_JOINT_SPEED;
constexpr float MAX_VELOCITY_GRIPPER_TILT = MAX_JOINT_SPEED;
constexpr float MAX_VELOCITY_GRIPPER_ROT = MAX_JOINT_SPEED;
constexpr float MAX_VELOCITY_GRIPPER_CLOSE = MAX_LIN_SPEED;

// Joints length, see doc or motion genesis code for bases reference
constexpr float J0x = 0.0f;
constexpr float J0y = 0.0f;
constexpr float J0z = 0.0f;

constexpr float J1x = 0.0f;
constexpr float J1y = 0.0f;
constexpr float J1z = 0.0f;

constexpr float J2x = 0.65f;
constexpr float J2y = 0.0f;
constexpr float J2z = 0.0f;

constexpr float J3x = 0.62f;
constexpr float J3y = 0.0f;
constexpr float J3z = 0.0f;

constexpr float J4x = 0.217f;
constexpr float J4y = 0.0f;
constexpr float J4z = 0.0f;

#endif // __ROBOT_CONFIGURATION__
