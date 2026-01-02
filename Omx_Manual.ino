#include "utils.h"

uint8_t flag = 0;

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
  if (Serial.available())
  {
    String s = Serial.readStringUntil('\n');
    float x, y, z;
    if (sscanf(s.c_str(), "%f,%f,%f", &x, &y, &z) == 3)
    {
      Serial.print("[CMD] Target: ");
      Serial.println(s);

      // 카메라 좌표를 매니퓰레이터 좌표계로 보정할 수 있다면 여기서 보정 (예: x += offset)
      moveTCPAbs(x, y, z);  // pitch은 0 도 기본값
    }
  }
}



