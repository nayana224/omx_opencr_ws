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
    이 파일은 함수 실습용 스케치입니다.
    아래 예시를 하나씩 주석 해제해서 정기구학/역기구학 동작을 관찰하세요.

    readJoint();
    readTCP();

    moveHome();
    moveJointAbs(0, 20, -20, 0, 2.0);
    moveJointRel(0, 10, 0, 0, 1.5);

    moveTCPAbs(0.20, 0.00, 0.10, 2.0);
    moveTCPRel(0.00, 0.00, 0.03, 1.5);

    keepHorizontal();
    setPitch(15);
    setGripper(true);
  */
}

