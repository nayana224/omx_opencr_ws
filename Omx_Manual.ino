#include "utils.h"

void setup()
{
  Serial.begin(115200);
  delay(1000);

  setPins();
  initManipulator();
  Serial.println("==== OpenManipulator Started! ====");
}

void loop()
{
  /*
    // 상태 확인
    readJoint();
    readTCP();

    // Joint 제어 [degree]
    moveHome();
    moveJointAbs(0, 20, -20, 0);
    moveJointRel(0, 10, 0, 0);

    // TCP 직선 이동 [m]
    moveTCPAbs(0.20, 0.00, 0.10);
    moveTCPRel(0.00, 0.00, 0.03);

    // End-effector 자세 / Gripper 제어
    keepHorizontal();
    setPitch(15);
    openGripper();
    closeGripper();

    // 단일 시퀀스 종료
    while (1);
  */
}
