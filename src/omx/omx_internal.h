#ifndef OMX_INTERNAL_H
#define OMX_INTERNAL_H

#include <open_manipulator_libs.h>
#include <Eigen.h>
#include <math.h>
#include <vector>

#include "omx_config.h"

OpenManipulator omx;        // 실제 OpenCR/DYNAMIXEL 제어 모델
OpenManipulator omx_model;  // 보정 좌표 기반 FK/IK 모델

inline double omxNowSec()
{
  return millis() / 1000.0;
}

inline double motionSpeedScale()
{
  if (OMX_MOTION_SPEED_SCALE < 0.1)
    return 0.1;
  if (OMX_MOTION_SPEED_SCALE > 1.0)
    return 1.0;
  return OMX_MOTION_SPEED_SCALE;
}

// 속도 상한을 만족하도록 요청 시간을 필요 시 연장합니다.
inline double resolveMoveTime(double requested_sec, double distance, double max_speed)
{
  double move_time = requested_sec;
  if (move_time < OMX_MIN_MOVE_SEC)
    move_time = OMX_MIN_MOVE_SEC;

  const double scaled_speed = max_speed * motionSpeedScale();
  if (scaled_speed > 0.0)
  {
    const double required_sec = distance / scaled_speed;
    if (required_sec > move_time)
      move_time = required_sec;
  }

  if (move_time > requested_sec + 0.001)
  {
    Serial.print("[SAFE] Move time adjusted to ");
    Serial.print(move_time, 2);
    Serial.println(" sec.");
  }

  return move_time;
}

// Raw actuator 좌표 -> 보정 좌표
inline std::vector<robotis_manipulator::JointValue> rawToCalibrated(
    const std::vector<robotis_manipulator::JointValue>& raw)
{
  std::vector<robotis_manipulator::JointValue> calibrated = raw;
  const size_t count = calibrated.size() < 4 ? calibrated.size() : 4;

  for (size_t i = 0; i < count; ++i)
    calibrated[i].position = raw[i].position - OMX_JOINT_ZERO_OFFSET_RAD[i];

  return calibrated;
}

// 보정 좌표 -> Raw actuator 좌표
inline std::vector<robotis_manipulator::JointValue> calibratedToRaw(
    const std::vector<robotis_manipulator::JointValue>& calibrated)
{
  std::vector<robotis_manipulator::JointValue> raw = calibrated;
  const size_t count = raw.size() < 4 ? raw.size() : 4;

  for (size_t i = 0; i < count; ++i)
    raw[i].position = calibrated[i].position + OMX_JOINT_ZERO_OFFSET_RAD[i];

  return raw;
}

// Actuator feedback과 보정 FK 상태를 동기화합니다.
inline bool syncRobotState(
    std::vector<robotis_manipulator::JointValue>* calibrated_out = nullptr,
    std::vector<robotis_manipulator::JointValue>* raw_out = nullptr)
{
  std::vector<robotis_manipulator::JointValue> raw = omx.receiveAllJointActuatorValue();

  if (raw.size() < 4)
  {
    Serial.println("[ERROR] Joint feedback unavailable.");
    return false;
  }

  std::vector<robotis_manipulator::JointValue> calibrated = rawToCalibrated(raw);
  omx_model.getManipulator()->setAllActiveJointValue(calibrated);
  omx_model.solveForwardKinematics();

  if (calibrated_out != nullptr)
    *calibrated_out = calibrated;
  if (raw_out != nullptr)
    *raw_out = raw;

  return true;
}

// Joint trajectory 시작점은 실제 위치만 유지하고 dynamic 값을 0으로 정규화합니다.
inline std::vector<robotis_manipulator::JointValue> stationaryJointWaypoint(
    const std::vector<robotis_manipulator::JointValue>& source)
{
  std::vector<robotis_manipulator::JointValue> waypoint = source;

  for (size_t i = 0; i < waypoint.size(); ++i)
  {
    waypoint[i].velocity = 0.0;
    waypoint[i].acceleration = 0.0;
    waypoint[i].effort = 0.0;
  }

  return waypoint;
}

// ROBOTIS control loop을 절대시간 기준으로 실행합니다.
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

  omx.processOpenManipulator(omxNowSec());
}

// 보정 좌표 기준 Joint 안전 범위를 검사합니다.
inline bool calibratedJointGoalIsSafe(
    const std::vector<robotis_manipulator::JointValue>& calibrated_goal)
{
  if (calibrated_goal.size() < 4)
    return false;

  for (size_t i = 0; i < 4; ++i)
  {
    const double deg = fabs(calibrated_goal[i].position * OMX_RAD_TO_DEG);
    if (deg > OMX_STUDENT_JOINT_LIMIT_DEG)
    {
      Serial.print("[ERROR] J");
      Serial.print(i + 1);
      Serial.println(" exceeds student safety range (-90 ~ +90 deg).");
      return false;
    }
  }

  return true;
}

// Raw 좌표 기준 ROBOTIS Joint limit을 검사합니다.
inline bool rawJointGoalIsSafe(
    const std::vector<robotis_manipulator::JointValue>& raw_goal)
{
  if (raw_goal.size() < 4)
    return false;

  std::vector<robotis_manipulator::Name> names =
      omx.getManipulator()->getAllActiveJointComponentName();

  if (!omx.checkJointLimit(names, raw_goal))
  {
    Serial.println("[ERROR] Goal exceeds ROBOTIS joint limit.");
    return false;
  }

  return true;
}

// 보정 좌표 기반 Joint trajectory [rad]
inline bool moveCalibratedJointRad(
    const std::vector<double>& calibrated_goal_rad,
    double requested_time)
{
  if (calibrated_goal_rad.size() < 4)
  {
    Serial.println("[ERROR] Invalid joint command.");
    return false;
  }

  std::vector<robotis_manipulator::JointValue> calibrated_present;
  std::vector<robotis_manipulator::JointValue> raw_present;
  if (!syncRobotState(&calibrated_present, &raw_present))
    return false;

  double max_delta_deg = 0.0;
  std::vector<robotis_manipulator::JointValue> calibrated_goal(4);

  for (size_t i = 0; i < 4; ++i)
  {
    calibrated_goal[i].position = calibrated_goal_rad[i];
    calibrated_goal[i].velocity = 0.0;
    calibrated_goal[i].acceleration = 0.0;
    calibrated_goal[i].effort = 0.0;

    const double delta_deg =
        fabs((calibrated_goal[i].position - calibrated_present[i].position)
             * OMX_RAD_TO_DEG);
    if (delta_deg > max_delta_deg)
      max_delta_deg = delta_deg;
  }

  if (!calibratedJointGoalIsSafe(calibrated_goal))
    return false;

  std::vector<robotis_manipulator::JointValue> raw_goal =
      calibratedToRaw(calibrated_goal);

  if (!rawJointGoalIsSafe(raw_goal))
    return false;

  const double move_time = resolveMoveTime(
      requested_time,
      max_delta_deg,
      OMX_MAX_JOINT_SPEED_DEG_S);

  const std::vector<robotis_manipulator::JointValue> raw_start =
      stationaryJointWaypoint(raw_present);

  omx.processOpenManipulator(omxNowSec());
  omx.makeJointTrajectory(raw_goal, move_time, raw_start);
  runManipulator(move_time + OMX_MOTION_SETTLE_SEC);

  return syncRobotState();
}

// 현재 tool orientation을 유지하는 Cartesian 직선 trajectory를 생성합니다.
inline bool startCalibratedLinearTaskTrajectory(
    const Eigen::Vector3d& target,
    double requested_time,
    bool relative,
    double* actual_move_time)
{
  std::vector<robotis_manipulator::JointValue> calibrated_present;
  if (!syncRobotState(&calibrated_present))
    return false;

  const Eigen::Vector3d current =
      omx_model.getKinematicPose("gripper").position;

  Eigen::Vector3d goal = target;
  if (relative)
    goal = current + target;

  const double distance = (goal - current).norm();
  const double move_time = resolveMoveTime(
      requested_time,
      distance,
      OMX_MAX_TCP_SPEED_M_S);

  const double now = omxNowSec();
  omx_model.getJointGoalValueFromTrajectory(now);
  omx_model.getTrajectory()->setPresentTime(now);

  omx_model.makeTaskTrajectory(
      "gripper", goal, move_time, calibrated_present);

  if (actual_move_time != nullptr)
    *actual_move_time = move_time;

  return true;
}

// 보정된 task-space trajectory를 실제 actuator에 적용합니다.
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
      if (!calibratedJointGoalIsSafe(calibrated_goal))
        return false;

      std::vector<robotis_manipulator::JointValue> raw_goal =
          calibratedToRaw(calibrated_goal);

      if (!rawJointGoalIsSafe(raw_goal))
        return false;

      if (!omx.sendAllJointActuatorValue(raw_goal))
      {
        Serial.println("[ERROR] Failed to send joint command.");
        return false;
      }
    }
    else
    {
      const double elapsed = (millis() - start_ms) / 1000.0;
      if (elapsed + 0.05 < move_time)
      {
        Serial.println("[ERROR] TCP trajectory stopped. Check IK/workspace.");
        return false;
      }
    }

    omx.receiveAllJointActuatorValue();
    delay(OMX_CONTROL_PERIOD_MS);
  }

  return syncRobotState();
}

inline bool initRobotRuntime()
{
  omx.initOpenManipulator(true);
  omx_model.initOpenManipulator(false);

  const double now = omxNowSec();
  omx.processOpenManipulator(now);
  omx_model.getJointGoalValueFromTrajectory(now);

  return syncRobotState();
}

#endif
