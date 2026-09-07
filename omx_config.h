#ifndef OMX_CONFIG_H
#define OMX_CONFIG_H

#include <stdint.h>
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

constexpr double OMX_DEG_TO_RAD = M_PI / 180.0;
constexpr double OMX_RAD_TO_DEG = 180.0 / M_PI;

// -----------------------------------------------------------------------------
// Software joint-zero calibration
// -----------------------------------------------------------------------------
// 학생/기구학 좌표 q_calibrated 와 실제 actuator 좌표 q_raw의 관계:
//
//   q_raw = q_calibrated + OMX_JOINT_ZERO_OFFSET_RAD[i]
//
// 현재 실물 로봇에서 J2/J3의 0도 자세 오차를 보정하기 위해 -4.8 deg를 사용합니다.
// DYNAMIXEL Homing Offset을 사용하지 않아도 되도록 모든 변환을 software layer에서
// 일관되게 처리합니다.
constexpr double OMX_JOINT_ZERO_OFFSET_RAD[4] = {
  0.0,
  -4.8 * OMX_DEG_TO_RAD,
  -4.8 * OMX_DEG_TO_RAD,
  0.0
};

// 실물 end-effector 수평 오차에 대한 추가 경험적 보정값.
// Joint zero calibration과 목적이 다르므로 별도로 유지합니다.
constexpr double OMX_PITCH_CORRECTION_RAD = -0.14;

// OpenManipulator control loop
constexpr uint32_t OMX_CONTROL_PERIOD_MS = 10;  // 100 Hz
constexpr double OMX_MOTION_SETTLE_SEC = 0.25;

// ROBOTIS OpenManipulator-X gripper tool limits
constexpr double OMX_GRIPPER_OPEN_M = 0.010;
constexpr double OMX_GRIPPER_CLOSE_M = -0.010;

// 대회용 외부 LED / Switch 보드 GPIO. OpenCR 내장 LED/SW 핀이 아닙니다.
const uint8_t LED_PIN[4] = {60, 61, 62, 63};
const uint8_t SW_PIN[4] = {50, 51, 52, 53};

#endif
