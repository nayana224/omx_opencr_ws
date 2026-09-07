#ifndef OMX_INTERNAL_H
#define OMX_INTERNAL_H

#include <open_manipulator_libs.h>
#include <Eigen/Dense>
#include <vector>

#include "omx_config.h"

// 실제 OpenCR/DYNAMIXEL을 제어하는 ROBOTIS 객체.
OpenManipulator omx;

// Software calibration이 적용된 joint 좌표로 FK/IK를 계산하는 ROBOTIS 모델.
// 실제 actuator에는 연결하지 않습니다.
OpenManipulator omx_model;

inline double omxNowSec()
{
  return millis() / 1000.0;
}

inline std::vector<robotis_manipulator::JointValue> rawToCalibrated(
    const std::vector<robotis_manipulator::JointValue>& raw)
{
  std::vector<robotis_manipulator::JointValue> calibrated = raw;
  const size_t count = calibrated.size() < 4 ? calibrated.size() : 4;

  for (size_t i = 0; i < count; ++i)
    calibrated[i].position = raw[i].position - OMX_JOINT_ZERO_OFFSET_RAD[i];

  return calibrated;
}

inline std::vector<robotis_manipulator::JointValue> calibratedToRaw(
    const std::vector<robotis_manipulator::JointValue>& calibrated)
{
  std::vector<robotis_manipulator::JointValue> raw = calibrated;
  const size_t count = raw.size() < 4 ? raw.size() : 4;

  for (size_t i = 0; i < count; ++i)
    raw[i].position = calibrated[i].position + OMX_JOINT_ZERO_OFFSET_RAD[i];

  // offset은 상수이므로 velocity/acceleration은 그대로 사용합니다.
  return raw;
}

// 실제 actuator 상태를 읽고, 같은 상태를 calibrated kinematics model에 반영합니다.
inline bool syncRobotState(std::vector<robotis_manipulator::JointValue>* calibrated_out = nullptr)
{
  std::vector<robotis_manipulator::JointValue> raw = omx.receiveAllJointActuatorValue();

  if (raw.size() < 4)
  {
    Serial.println("[ERROR] OpenManipulator joint feedback unavailable.");
    return false;
  }

  std::vector<robotis_manipulator::JointValue> calibrated = rawToCalibrated(raw);

  omx_model.getManipulator()->setAllActiveJointValue(calibrated);
  omx_model.solveForwardKinematics();

  if (calibrated_out != nullptr)
    *calibrated_out = calibrated;

  return true;
}

// ROBOTIS control loop은 부팅 이후 계속 증가하는 절대 시간을 받아야 합니다.
inline void runManipulator(double sec)
{
  if (sec <= 0.0)
    return;

  const uint32_t duration_ms = static_cast<uint32_t>(sec * 1000.0);
  const uint32_t start_ms = millis();

  while ((uint32_t)(millis() - start_ms) < duration_ms)
  {
    omx.processOpenManipulator(omxNowSec());
    delay(OMX_CONTROL_PERIOD_MS);
  }

  // 마지막 상태를 한 번 더 반영해 readJoint/readTCP가 최신 상태를 보도록 합니다.
  omx.processOpenManipulator(omxNowSec());
}

inline bool rawJointGoalIsSafe(const std::vector<robotis_manipulator::JointValue>& raw_goal)
{
  if (raw_goal.size() < 4)
    return false;

  std::vector<robotis_manipulator::Name> names =
      omx.getManipulator()->getAllActiveJointComponentName();

  if (!omx.checkJointLimit(names, raw_goal))
  {
    Serial.println("[ERROR] Joint goal exceeds OpenManipulator-X limit.");
    return false;
  }

  return true;
}

inline bool moveCalibratedJointRad(const std::vector<double>& calibrated_goal_rad, double move_time)
{
  if (calibrated_goal_rad.size() < 4 || move_time <= 0.0)
  {
    Serial.println("[ERROR] Invalid joint command.");
    return false;
  }

  // 새 trajectory를 만들기 직전에 ROBOTIS clock을 현재 절대시간으로 갱신합니다.
  omx.processOpenManipulator(omxNowSec());

  // calibrated TCP trajectory를 low-level actuator 전송으로 수행한 뒤에도
  // 다음 joint trajectory가 stale trajectory state가 아닌 실제 feedback에서 시작하도록
  // 현재 raw joint를 명시적인 present_joint_value로 넘깁니다.
  std::vector<robotis_manipulator::JointValue> raw_present =
      omx.receiveAllJointActuatorValue();

  if (raw_present.size() < 4)
  {
    Serial.println("[ERROR] Cannot start joint trajectory without joint feedback.");
    return false;
  }

  std::vector<robotis_manipulator::JointValue> calibrated_goal(4);
  for (size_t i = 0; i < 4; ++i)
  {
    calibrated_goal[i].position = calibrated_goal_rad[i];
    calibrated_goal[i].velocity = 0.0;
    calibrated_goal[i].acceleration = 0.0;
    calibrated_goal[i].effort = 0.0;
  }

  std::vector<robotis_manipulator::JointValue> raw_goal =
      calibratedToRaw(calibrated_goal);

  if (!rawJointGoalIsSafe(raw_goal))
    return false;

  omx.makeJointTrajectory(raw_goal, move_time, raw_present);
  runManipulator(move_time + OMX_MOTION_SETTLE_SEC);
  return syncRobotState();
}

inline bool startCalibratedTaskTrajectory(
    const Eigen::Vector3d& goal_position,
    double move_time,
    bool relative)
{
  if (move_time <= 0.0)
  {
    Serial.println("[ERROR] TCP move time must be positive.");
    return false;
  }

  std::vector<robotis_manipulator::JointValue> calibrated_present;
  if (!syncRobotState(&calibrated_present))
    return false;

  // Shadow model의 trajectory storage를 최초 한 번 ROBOTIS 모델로 초기화하고,
  // 현재 절대 시간을 trajectory start time의 기준으로 맞춥니다.
  omx_model.getJointGoalValueFromTrajectory(omxNowSec());
  omx_model.getTrajectory()->setPresentTime(omxNowSec());

  if (relative)
  {
    omx_model.makeTaskTrajectoryFromPresentPose(
        "gripper", goal_position, move_time, calibrated_present);
  }
  else
  {
    omx_model.makeTaskTrajectory(
        "gripper", goal_position, move_time, calibrated_present);
  }

  return true;
}

// Shadow ROBOTIS model이 만든 calibrated task trajectory를 실제 raw actuator 좌표로
// 변환해 전송합니다. FK/IK와 minimum-jerk trajectory는 ROBOTIS 구현을 그대로 사용합니다.
inline bool runCalibratedTaskTrajectory(double move_time)
{
  if (move_time <= 0.0)
    return false;

  const uint32_t start_ms = millis();
  const uint32_t duration_ms =
      static_cast<uint32_t>((move_time + OMX_MOTION_SETTLE_SEC) * 1000.0);

  while ((uint32_t)(millis() - start_ms) < duration_ms)
  {
    const double now = omxNowSec();
    std::vector<robotis_manipulator::JointValue> calibrated_goal =
        omx_model.getJointGoalValueFromTrajectory(now);

    if (!calibrated_goal.empty())
    {
      std::vector<robotis_manipulator::JointValue> raw_goal =
          calibratedToRaw(calibrated_goal);

      if (!rawJointGoalIsSafe(raw_goal))
        return false;

      if (!omx.sendAllJointActuatorValue(raw_goal))
      {
        Serial.println("[ERROR] Failed to send OpenManipulator joint command.");
        return false;
      }
    }
    else
    {
      const double elapsed = (millis() - start_ms) / 1000.0;
      if (elapsed + 0.05 < move_time)
      {
        Serial.println("[ERROR] TCP trajectory stopped early. Check IK/workspace.");
        return false;
      }
    }

    // 공식 processOpenManipulator와 동일하게 실제 joint feedback을 주기적으로 갱신합니다.
    omx.receiveAllJointActuatorValue();
    delay(OMX_CONTROL_PERIOD_MS);
  }

  return syncRobotState();
}

inline bool initRobotRuntime()
{
  omx.initOpenManipulator(true);
  omx_model.initOpenManipulator(false);

  // 두 ROBOTIS trajectory 객체 모두 절대 시간 기준을 초기화합니다.
  omx.processOpenManipulator(omxNowSec());
  omx_model.getJointGoalValueFromTrajectory(omxNowSec());

  return syncRobotState();
}

#endif
