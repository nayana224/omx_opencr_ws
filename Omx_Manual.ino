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
    학생 실습용 예시입니다.
    필요한 함수 한 줄씩 주석을 해제해서 동작을 관찰하세요.

    // 상태 읽기
    readJoint();
    readTCP();

    // Joint 제어
    moveHome();
    moveJointAbs(0, 20, -20, 0, 2.0);
    moveJointRel(0, 10, 0, 0, 1.5);

    // TCP 제어
    moveTCPAbs(0.20, 0.00, 0.10, 2.0);
    moveTCPRel(0.00, 0.00, 0.03, 1.5);

    // 자세 / Gripper
    keepHorizontal();
    setPitch(15);
    openGripper();
    closeGripper();

    // 한 번만 실행하려면 마지막에 사용
    while (1);
  */
}
