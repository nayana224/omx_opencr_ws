#ifndef UTILS_HPP
#define UTILS_HPP

#include <Eigen/Dense>
#include <math.h>
#include <vector>

#include "src/omx/omx_internal.h"

inline void initManipulator()
{
  if (initRobotRuntime())
    Serial.println("[OK] OpenManipulator initialized.");
  else
    Serial.println("[ERROR] Joint feedback sync failed.");
}

// 현재 Joint 각도 [degree]
inline std::vector<double> readJoint()
{
  std::vector<robotis_manipulator::JointValue> joints;
  if (!syncRobotState(&joints))
    return {};

  std::vector<double> joint_deg(4);
  Serial.println("[INFO] Joint angles (deg)");

  for (int i = 0; i < 4; ++i)
  {
    joint_deg[i] = joints[i].position * OMX_RAD_TO_DEG;
    Serial.print("  J");
    Serial.print(i + 1);
    Serial.print(": ");
    Serial.println(joint_deg[i], 2);
  }

  return joint_deg;
}

// BASE 좌표 기준 현재 TCP 위치 [m]
inline Eigen::Vector3d readTCP()
{
  if (!syncRobotState())
    return Eigen::Vector3d(NAN, NAN, NAN);

  const Eigen::Vector3d tcp =
      omx_model.getKinematicPose("gripper").position;

  Serial.println("[INFO] TCP position (m)");
  Serial.print("  X: "); Serial.println(tcp(0), 6);
  Serial.print("  Y: "); Serial.println(tcp(1), 6);
  Serial.print("  Z: "); Serial.println(tcp(2), 6);

  return tcp;
}

// Home 자세 이동 [degree]
inline void moveHome(double t = OMX_DEFAULT_MOVE_SEC)
{
  moveCalibratedJointRad({0.0, 0.0, 0.0, 0.0}, t);
}

// Joint 절대 이동 [degree]
inline void moveJointAbs(
    float j1, float j2, float j3, float j4,
    double t = OMX_DEFAULT_MOVE_SEC)
{
  moveCalibratedJointRad({
    j1 * OMX_DEG_TO_RAD,
    j2 * OMX_DEG_TO_RAD,
    j3 * OMX_DEG_TO_RAD,
    j4 * OMX_DEG_TO_RAD
  }, t);
}

// Joint 상대 이동 [degree]
inline void moveJointRel(
    float dj1, float dj2, float dj3, float dj4,
    double t = OMX_DEFAULT_MOVE_SEC)
{
  std::vector<robotis_manipulator::JointValue> present;
  if (!syncRobotState(&present))
    return;

  moveCalibratedJointRad({
    present[0].position + dj1 * OMX_DEG_TO_RAD,
    present[1].position + dj2 * OMX_DEG_TO_RAD,
    present[2].position + dj3 * OMX_DEG_TO_RAD,
    present[3].position + dj4 * OMX_DEG_TO_RAD
  }, t);
}

// BASE 좌표 기준 TCP 절대 위치 직선 이동 [m]
inline void moveTCPAbs(
    float x, float y, float z,
    double t = OMX_DEFAULT_MOVE_SEC)
{
  double move_time = t;
  if (!startCalibratedLinearTaskTrajectory(
          Eigen::Vector3d(x, y, z), t, false, &move_time))
    return;

  runCalibratedTaskTrajectory(move_time);
}

// 현재 TCP 기준 상대 위치 직선 이동 [m]
inline void moveTCPRel(
    float dx, float dy, float dz,
    double t = OMX_DEFAULT_MOVE_SEC)
{
  double move_time = t;
  if (!startCalibratedLinearTaskTrajectory(
          Eigen::Vector3d(dx, dy, dz), t, true, &move_time))
    return;

  runCalibratedTaskTrajectory(move_time);
}

// Gripper open/close 명령
inline void setGripper(bool open, double wait_sec = 1.0)
{
  if (wait_sec < OMX_MIN_MOVE_SEC)
    wait_sec = OMX_MIN_MOVE_SEC;

  omx.processOpenManipulator(omxNowSec());
  omx.makeToolTrajectory(
      "gripper",
      open ? OMX_GRIPPER_OPEN_M : OMX_GRIPPER_CLOSE_M);

  runManipulator(wait_sec + OMX_MOTION_SETTLE_SEC);
}

inline void openGripper(double wait_sec = 1.0)
{
  setGripper(true, wait_sec);
}

inline void closeGripper(double wait_sec = 1.0)
{
  setGripper(false, wait_sec);
}

// 바닥 기준 수평 자세
inline void keepHorizontal(double t = OMX_DEFAULT_MOVE_SEC)
{
  std::vector<robotis_manipulator::JointValue> joints;
  if (!syncRobotState(&joints))
    return;

  const double j4 =
      -(joints[1].position + joints[2].position)
      + OMX_PITCH_CORRECTION_RAD;

  moveCalibratedJointRad({
    joints[0].position,
    joints[1].position,
    joints[2].position,
    j4
  }, t);
}

// End-effector 절대 pitch [degree]
inline void setPitch(
    double target_pitch_deg,
    double t = OMX_DEFAULT_PITCH_MOVE_SEC)
{
  std::vector<robotis_manipulator::JointValue> joints;
  if (!syncRobotState(&joints))
    return;

  const double target_pitch = target_pitch_deg * OMX_DEG_TO_RAD;
  const double j4 =
      target_pitch
      - (joints[1].position + joints[2].position)
      + OMX_PITCH_CORRECTION_RAD;

  moveCalibratedJointRad({
    joints[0].position,
    joints[1].position,
    joints[2].position,
    j4
  }, t);
}

// 대회용 외부 I/O 보드
inline void setPins()
{
  for (int i = 0; i < 4; ++i)
  {
    pinMode(LED_PIN[i], OUTPUT);
    pinMode(SW_PIN[i], INPUT);
  }
}

inline void setLEDs(uint8_t value)
{
  value &= 0x0F;
  for (uint8_t i = 0; i < 4; ++i)
    digitalWrite(LED_PIN[i], (value & (1 << i)) ? HIGH : LOW);
}

inline int SW1() { return digitalRead(SW_PIN[0]); }
inline int SW2() { return digitalRead(SW_PIN[1]); }
inline int SW3() { return digitalRead(SW_PIN[2]); }
inline int SW4() { return digitalRead(SW_PIN[3]); }

#endif
