#ifndef UTILS_HPP
#define UTILS_HPP

#include <Eigen/Dense>
#include <math.h>
#include <vector>

#include "omx_internal.h"

// ==================== 초기화 ====================
inline void initManipulator()
{
  if (initRobotRuntime())
    Serial.println("[OK] OpenManipulator initialized and joint feedback synced.");
  else
    Serial.println("[ERROR] OpenManipulator initialization finished, but joint feedback sync failed.");
}

// ==================== readJoint(): 현재 조인트 각도 읽기 ====================
// 학생에게는 software calibration이 적용된 J1~J4 각도를 degree로 보여줍니다.
inline std::vector<double> readJoint()
{
  std::vector<robotis_manipulator::JointValue> joints;
  if (!syncRobotState(&joints))
    return {};

  std::vector<double> jnt_deg(4);
  for (int i = 0; i < 4; ++i)
    jnt_deg[i] = joints[i].position * OMX_RAD_TO_DEG;

  Serial.println("[INFO] readJoint(): Current calibrated joint angles (deg)");
  for (int i = 0; i < 4; ++i)
  {
    Serial.print("  J"); Serial.print(i + 1);
    Serial.print(": "); Serial.print(jnt_deg[i], 2);
    Serial.println(" deg");
  }

  return jnt_deg;
}

// ==================== readTCP(): 현재 TCP 좌표 읽기 ====================
// 실제 actuator 값을 읽은 뒤 calibrated ROBOTIS model로 FK를 계산합니다.
inline Eigen::Vector3d readTCP()
{
  if (!syncRobotState())
    return Eigen::Vector3d(NAN, NAN, NAN);

  robotis_manipulator::KinematicPose pose = omx_model.getKinematicPose("gripper");
  Eigen::Vector3d tcp = pose.position;

  Serial.println("[INFO] readTCP(): Current calibrated TCP position (m)");
  Serial.print("  X: "); Serial.println(tcp(0), 6);
  Serial.print("  Y: "); Serial.println(tcp(1), 6);
  Serial.print("  Z: "); Serial.println(tcp(2), 6);

  return tcp;
}

// ==================== moveHome(): calibrated 0 deg 자세 ====================
inline void moveHome(double t = 2.0)
{
  moveCalibratedJointRad({0.0, 0.0, 0.0, 0.0}, t);
}

// ==================== moveJointAbs(): Joint 절대이동 [degree] ====================
inline void moveJointAbs(float j1, float j2, float j3, float j4, double t = 2.0)
{
  std::vector<double> goal_rad = {
    j1 * OMX_DEG_TO_RAD,
    j2 * OMX_DEG_TO_RAD,
    j3 * OMX_DEG_TO_RAD,
    j4 * OMX_DEG_TO_RAD
  };

  moveCalibratedJointRad(goal_rad, t);
}

// ==================== moveJointRel(): Joint 상대이동 [degree] ====================
inline void moveJointRel(float dj1, float dj2, float dj3, float dj4, double t = 2.0)
{
  std::vector<robotis_manipulator::JointValue> present;
  if (!syncRobotState(&present))
    return;

  std::vector<double> goal_rad = {
    present[0].position + dj1 * OMX_DEG_TO_RAD,
    present[1].position + dj2 * OMX_DEG_TO_RAD,
    present[2].position + dj3 * OMX_DEG_TO_RAD,
    present[3].position + dj4 * OMX_DEG_TO_RAD
  };

  moveCalibratedJointRad(goal_rad, t);
}

// ==================== moveTCPAbs(): TCP 절대 위치 직선이동 [m] ====================
// ROBOTIS task trajectory + IK를 calibrated shadow model에서 그대로 사용합니다.
inline void moveTCPAbs(float x, float y, float z, double t = 2.0)
{
  Eigen::Vector3d pos(x, y, z);

  if (!startCalibratedTaskTrajectory(pos, t, false))
    return;

  runCalibratedTaskTrajectory(t);
}

// ==================== moveTCPRel(): TCP 상대이동 [m] ====================
inline void moveTCPRel(float dx, float dy, float dz, double t = 2.0)
{
  Eigen::Vector3d delta(dx, dy, dz);

  if (!startCalibratedTaskTrajectory(delta, t, true))
    return;

  runCalibratedTaskTrajectory(t);
}

// ==================== setGripper(): 그리퍼 제어 ====================
inline void setGripper(bool open, double wait_sec = 1.0)
{
  // tool trajectory는 별도 move_time을 받지 않으므로 wait_sec은 명령 후 대기시간입니다.
  omx.processOpenManipulator(omxNowSec());

  const double goal = open ? OMX_GRIPPER_OPEN_M : OMX_GRIPPER_CLOSE_M;
  omx.makeToolTrajectory("gripper", goal);
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

// ==================== keepHorizontal(): 수평유지 보정 ====================
// OpenManipulator-X의 J2/J3/J4는 같은 Y축 회전이므로
// 기본 pitch = J2 + J3 + J4 관계를 이용합니다.
inline void keepHorizontal(double t = 2.0)
{
  std::vector<robotis_manipulator::JointValue> joints;
  if (!syncRobotState(&joints))
    return;

  const double j4_new =
      -(joints[1].position + joints[2].position) + OMX_PITCH_CORRECTION_RAD;

  std::vector<double> goal = {
    joints[0].position,
    joints[1].position,
    joints[2].position,
    j4_new
  };

  moveCalibratedJointRad(goal, t);
}

// ==================== setPitch(): 지정 pitch로 기울이기 ====================
// pitch = J2 + J3 + J4 -> J4 = pitch - (J2 + J3)
inline void setPitch(double target_pitch_deg, double t = 1.5)
{
  std::vector<robotis_manipulator::JointValue> joints;
  if (!syncRobotState(&joints))
    return;

  const double target_pitch_rad = target_pitch_deg * OMX_DEG_TO_RAD;
  const double j4_new =
      target_pitch_rad - (joints[1].position + joints[2].position)
      + OMX_PITCH_CORRECTION_RAD;

  std::vector<double> goal = {
    joints[0].position,
    joints[1].position,
    joints[2].position,
    j4_new
  };

  moveCalibratedJointRad(goal, t);
}

// ==================== 외부 LED / SW 보드 ====================
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
