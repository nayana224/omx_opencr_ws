#ifndef UTILS_HPP
#define UTILS_HPP

#include <open_manipulator_libs.h>
#include <Eigen/Dense>
#include <math.h>
#include <vector>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

constexpr double OMX_DEG_TO_RAD = M_PI / 180.0;
constexpr double OMX_RAD_TO_DEG = 180.0 / M_PI;
constexpr float OMX_JOINT_OFFSET_DEG = -4.8; // Manual calibration offset for J2/J3.
constexpr double OMX_PITCH_OFFSET_RAD = -0.14;

const uint8_t LED_PIN[4] = {60, 61, 62, 63};
const uint8_t SW_PIN[4] = {50, 51, 52, 53};


OpenManipulator omx;

// ==================== 초기화 ====================
inline void initManipulator()
{
  omx.initOpenManipulator(true);
  Serial.println("[OK] Manipulator initialized.");
}

// ==================== 내부 제어 루프 실행 ====================
inline void runManipulator(double sec)
{
  if (sec <= 0.0)
    return;

  const uint32_t period_ms = 10; // 100 Hz
  uint32_t start_ms = millis();
  double start_time = start_ms / 1000.0;

  while ((millis() - start_ms) < (uint32_t)(sec * 1000.0))
  {
    double now = (millis() / 1000.0) - start_time;
    omx.processOpenManipulator(now);
    delay(period_ms);
  }
}


// ==================== readJoint(): 현재 조인트 각도 읽기 ====================
inline std::vector<double> readJoint()
{
  std::vector<robotis_manipulator::JointValue> joints = omx.getAllActiveJointValue();
  std::vector<double> jnt_deg(4);

  if (joints.size() < 4)
  {
    Serial.println("[ERROR] readJoint(): Could not read 4 active joints.");
    return jnt_deg;
  }

  for (int i = 0; i < 4; i++)
    jnt_deg[i] = joints[i].position * OMX_RAD_TO_DEG;  // rad -> deg 변환

  Serial.println("[INFO] readJoint(): Current joint angles (deg)");
  for (int i = 0; i < 4; i++)
  {
    Serial.print("  J"); Serial.print(i + 1);
    Serial.print(": "); Serial.print(jnt_deg[i], 2);
    Serial.println(" deg");
  }
  return jnt_deg;
}

// ==================== readTCP(): 현재 TCP 좌표 읽기 ====================
inline Eigen::Vector3d readTCP()
{
  robotis_manipulator::KinematicPose pose = omx.getKinematicPose("gripper");
  Eigen::Vector3d tcp = pose.position;

  Serial.println("[INFO] readTCP(): Current TCP position (m)");
  Serial.print("  X: "); Serial.println(tcp(0), 6);
  Serial.print("  Y: "); Serial.println(tcp(1), 6);
  Serial.print("  Z: "); Serial.println(tcp(2), 6);
  return tcp;
}

inline void moveHome(double t = 2.0)
{
  runManipulator(0.15);

  // 오프셋 보정(rad 단위)
  std::vector<double> home_pos = {
    0.0,
    OMX_JOINT_OFFSET_DEG * OMX_DEG_TO_RAD,
    OMX_JOINT_OFFSET_DEG * OMX_DEG_TO_RAD,
    0.0
  };
  omx.makeJointTrajectory(home_pos, t);

  runManipulator(t + 0.25);
}



// ==================== moveJointAbs(): Joint 절대이동 [degree] ====================
inline void moveJointAbs(float j1, float j2, float j3, float j4, double t = 2.0)
{
  runManipulator(0.15);

  // === 오프셋 보정 (J2, J3) ===
  j2 += OMX_JOINT_OFFSET_DEG;
  j3 += OMX_JOINT_OFFSET_DEG;

  // deg -> rad 변환
  std::vector<double> goal_rad = {
    j1 * OMX_DEG_TO_RAD,
    j2 * OMX_DEG_TO_RAD,
    j3 * OMX_DEG_TO_RAD,
    j4 * OMX_DEG_TO_RAD
  };

  omx.makeJointTrajectory(goal_rad, t);
  runManipulator(t + 0.25);
}


// ==================== moveJointRel(): Joint 상대이동 [degree] ====================
inline void moveJointRel(float dj1, float dj2, float dj3, float dj4, double t = 2.0)
{
  runManipulator(0.15);

  // deg -> rad 변환
  std::vector<double> delta_rad = {
    dj1 * OMX_DEG_TO_RAD,
    dj2 * OMX_DEG_TO_RAD,
    dj3 * OMX_DEG_TO_RAD,
    dj4 * OMX_DEG_TO_RAD
  };

  omx.makeJointTrajectoryFromPresentPosition(delta_rad, t);
  runManipulator(t + 0.25);
}

// ==================== moveTCPAbs(): TCP 절대 위치 직선이동 [m] ====================
inline void moveTCPAbs(float x, float y, float z, double t = 2.0)
{
  runManipulator(0.15);

  Eigen::Vector3d pos(x, y, z);
  omx.makeTaskTrajectory("gripper", pos, t);

  runManipulator(t + 0.25);
}

// ==================== moveTCPRel(): TCP 상대이동 [m] ====================
inline void moveTCPRel(float dx, float dy, float dz, double t = 2.0)
{
  runManipulator(0.15);
  Eigen::Vector3d delta(dx, dy, dz);
  omx.makeTaskTrajectoryFromPresentPose("gripper", delta, t);
  runManipulator(t + 0.25);
}

// ==================== setGripper(): 그리퍼 제어 ====================
inline void setGripper(bool open, double t = 1.0)
{
  runManipulator(0.15);
  double val = open ? +0.01 : -0.01;
  omx.makeToolTrajectory("gripper", val);
  Serial.println("[CMD] setGripper() - Gripper move.");
  runManipulator(t + 0.25);
}


// ==================== keepHorizontal(): 수평유지 보정 ====================
// 조인트 2, 3, 4의 합이 0이 되도록 조정 -> TCP가 수평을 유지하도록 함.
inline void keepHorizontal(double t = 2)
{
  runManipulator(0.15);
  omx.receiveAllJointActuatorValue(); // 최신 상태 반영

  std::vector<double> jnt = readJoint();  // [J1, J2, J3, J4] in deg

  // deg → rad 변환
  double j1 = jnt[0] * OMX_DEG_TO_RAD;
  double j2 = jnt[1] * OMX_DEG_TO_RAD;
  double j3 = jnt[2] * OMX_DEG_TO_RAD;

  // 수평 유지 계산 (rad) (약간의 보정 포함)
  double j4_new = -(j2 + j3) + OMX_PITCH_OFFSET_RAD;

  // rad 단위 goal
  std::vector<double> goal = {j1, j2, j3, j4_new};
  omx.makeJointTrajectory(goal, t);
  runManipulator(t + 0.25);
}

// ==================== setPitch(): 지정한 pitch각으로 기울이기 ====================
// pitch(rad) = J2 + J3 + J4  →  J4 = pitch - (J2 + J3)
inline void setPitch(double target_pitch_deg, double t = 1.5)
{
  runManipulator(0.15);
  omx.receiveAllJointActuatorValue(); // 최신 상태 반영
  // 현재 조인트 각도 읽기 (deg 단위)
  std::vector<double> jnt_deg = readJoint();  // [J1, J2, J3, J4]

  // deg → rad 변환
  double j1 = jnt_deg[0] * OMX_DEG_TO_RAD;
  double j2 = jnt_deg[1] * OMX_DEG_TO_RAD;
  double j3 = jnt_deg[2] * OMX_DEG_TO_RAD;
  double target_pitch_rad = target_pitch_deg * OMX_DEG_TO_RAD;

  // 지정된 pitch 각도에 맞게 J4 계산 (약간의 보정 포함)
  double j4_new = target_pitch_rad - (j2 + j3) + OMX_PITCH_OFFSET_RAD;

  // rad 단위 goal 벡터 생성
  std::vector<double> goal = {j1, j2, j3, j4_new};
  omx.makeJointTrajectory(goal, t);
  runManipulator(t + 0.25);
}

//======= LED, SW 관련 함수=============
inline void setPins()
{
  for (int i = 0; i < 4; i++) {
    pinMode(LED_PIN[i], OUTPUT);
    pinMode(SW_PIN[i], INPUT);
  }
}

inline void setLEDs(uint8_t value) {
  value &= 0x0F; // 하위 4비트만 사용
  for (uint8_t i = 0; i < 4; i++) {
    // i번째 비트가 1이면 켜고, 0이면 끔
    digitalWrite(LED_PIN[i], (value & (1 << i)) ? HIGH : LOW);
  }
}

inline int SW1() {
  return digitalRead(SW_PIN[0]);
}
inline int SW2() {
  return digitalRead(SW_PIN[1]);
}
inline int SW3() {
  return digitalRead(SW_PIN[2]);
}
inline int SW4() {
  return digitalRead(SW_PIN[3]);
}



#endif
