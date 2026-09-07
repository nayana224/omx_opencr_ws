#ifndef OMX_CONFIG_H
#define OMX_CONFIG_H

#include <stdint.h>
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

constexpr double OMX_DEG_TO_RAD = M_PI / 180.0;
constexpr double OMX_RAD_TO_DEG = 180.0 / M_PI;

// Joint zero 소프트웨어 보정값 [rad]
// q_raw = q_student + offset
constexpr double OMX_JOINT_ZERO_OFFSET_RAD[4] = {
  0.0,
  -4.8 * OMX_DEG_TO_RAD,
  -4.8 * OMX_DEG_TO_RAD,
  0.0
};

// End-effector pitch 보정값 [rad]
constexpr double OMX_PITCH_CORRECTION_RAD = -0.14;

// Motion 시간 설정
constexpr uint32_t OMX_CONTROL_PERIOD_MS = 10;  // 100 Hz
constexpr double OMX_MOTION_SETTLE_SEC = 0.25;
constexpr double OMX_MIN_MOVE_SEC = 1.0;
constexpr double OMX_DEFAULT_MOVE_SEC = 2.0;
constexpr double OMX_DEFAULT_PITCH_MOVE_SEC = 1.5;

// Joint/TCP 공통 속도 scale 및 개별 속도 상한
constexpr double OMX_MOTION_SPEED_SCALE = 1.0;
constexpr double OMX_MAX_JOINT_SPEED_DEG_S = 30.0;
constexpr double OMX_MAX_TCP_SPEED_M_S = 0.05;

// 학생용 Joint 명령 범위 [degree]
constexpr double OMX_STUDENT_JOINT_LIMIT_DEG = 90.0;

// ROBOTIS gripper tool 위치 한계 [m]
constexpr double OMX_GRIPPER_OPEN_M = 0.010;
constexpr double OMX_GRIPPER_CLOSE_M = -0.010;

// 대회용 외부 I/O 보드 GPIO
const uint8_t LED_PIN[4] = {60, 61, 62, 63};
const uint8_t SW_PIN[4] = {50, 51, 52, 53};

#endif
