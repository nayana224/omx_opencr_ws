#ifndef OMX_CONFIG_H
#define OMX_CONFIG_H

#include <stdint.h>
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

constexpr double OMX_DEG_TO_RAD = M_PI / 180.0;
constexpr double OMX_RAD_TO_DEG = 180.0 / M_PI;

// 학생 좌표와 실제 actuator 좌표의 zero 보정값.
// q_raw = q_student + offset
constexpr double OMX_JOINT_ZERO_OFFSET_RAD[4] = {
  0.0,
  -4.8 * OMX_DEG_TO_RAD,
  -4.8 * OMX_DEG_TO_RAD,
  0.0
};

// 실물 end-effector 수평 오차에 대한 추가 보정값.
constexpr double OMX_PITCH_CORRECTION_RAD = -0.14;

// 공통 motion 설정
constexpr uint32_t OMX_CONTROL_PERIOD_MS = 10;  // 100 Hz
constexpr double OMX_MOTION_SETTLE_SEC = 0.25;
constexpr double OMX_MIN_MOVE_SEC = 1.0;
constexpr double OMX_DEFAULT_MOVE_SEC = 2.0;
constexpr double OMX_DEFAULT_PITCH_MOVE_SEC = 1.5;

// Joint/TCP는 단위가 다르므로 각각 안전 상한을 두고 같은 scale을 적용합니다.
// 0.5로 낮추면 Joint와 TCP 모두 절반 속도 수준으로 동작합니다.
constexpr double OMX_MOTION_SPEED_SCALE = 1.0;
constexpr double OMX_MAX_JOINT_SPEED_DEG_S = 30.0;
constexpr double OMX_MAX_TCP_SPEED_M_S = 0.05;

// 대회 교육 기준의 추가 joint 안전 범위.
constexpr double OMX_STUDENT_JOINT_LIMIT_DEG = 90.0;

// ROBOTIS OpenManipulator-X gripper tool limits
constexpr double OMX_GRIPPER_OPEN_M = 0.010;
constexpr double OMX_GRIPPER_CLOSE_M = -0.010;

// 대회용 외부 LED / Switch 보드 GPIO
const uint8_t LED_PIN[4] = {60, 61, 62, 63};
const uint8_t SW_PIN[4] = {50, 51, 52, 53};

#endif
