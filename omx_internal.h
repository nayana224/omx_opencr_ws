#ifndef OMX_INTERNAL_H
#define OMX_INTERNAL_H

#include <open_manipulator_libs.h>
#include <Eigen/Dense>
#include <math.h>
#include <vector>

#include "omx_config.h"

OpenManipulator omx;        // 실제 OpenCR/DYNAMIXEL 제어
OpenManipulator omx_model;  // 보정 좌표용 ROBOTIS FK/IK 모델

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

  return raw;
}

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

  omx.processOpenManipulator(omxNowSec());
  omx.makeJointTrajectory(raw_goal, move_time, raw_present);
  runManipulator(move_time + OMX_MOTION_SETTLE_SEC);

  return syncRobotState();
}

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
  const Eigen::Vector3d goal = relative ? current + target : target;
  const double distance = (goal - current).norm();

  const double move_time = resolveMoveTime(
      requested_time,
      distance,
      OMX_MAX_TCP_SPEED_M_S);

  const double now = omxNowSec();
  omx_model.getJointGoalValueFromTrajectory(now);
  omx_model.getTrajectory()->setPresentTime(now);

  // Vector3d task trajectory는 현재 orientation을 유지한 채 XYZ를 직선 보간합니다.
  omx_model.makeTaskTrajectory(
      "gripper", goal, move_time, calibrated_present);

  if (actual_move_time != nullptr)
    *actual_move_time = move_time;

  return true;
}

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
