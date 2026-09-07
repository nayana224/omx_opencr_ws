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
    // 상태 읽기
    readJoint();
    readTCP();

    // Joint 제어
    moveHome();
    moveJointAbs(0, 20, -20, 0);
    moveJointRel(0, 10, 0, 0);

    // TCP 직선 이동
    moveTCPAbs(0.20, 0.00, 0.10);
    moveTCPRel(0.00, 0.00, 0.03);

    // 자세 / 그리퍼
    keepHorizontal();
    setPitch(15);
    openGripper();
    closeGripper();

    // 한 번만 실행
    while (1);
  */
}
