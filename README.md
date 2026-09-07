# OpenMANIPULATOR-X OpenCR Student API

ROBOTIS **OpenMANIPULATOR-X**를 OpenCR + Arduino로 쉽게 제어하기 위한 교육/대회용 코드입니다.

학생은 DYNAMIXEL 통신, radian 변환, FK/IK, `processOpenManipulator()` 같은 내부 제어를 직접 작성하지 않아도 됩니다.
**`Omx_Manual.ino`의 `loop()` 안에서 필요한 함수를 한 줄씩 호출하면 됩니다.**

---

## 1. 처음이라면 이것만 보면 됩니다

### 학생이 주로 수정하는 파일

```text
Omx_Manual.ino
```

기본 `setup()`은 이미 준비되어 있으므로 보통 `loop()`만 작성하면 됩니다.

```cpp
void loop()
{
  moveHome();
  moveJointAbs(0, 20, -20, 0);
  moveTCPRel(0.00, 0.00, 0.03);
  closeGripper();

  while (1);  // 한 번만 실행
}
```

### 꼭 기억할 단위

| 항목 | 단위 |
| --- | --- |
| Joint 각도 | `degree` |
| TCP 위치 | `meter` |
| 동작 시간 `t` | `second` |

예를 들어:

```cpp
moveJointAbs(0, 30, -20, 0);   // degree
moveTCPRel(0, 0, 0.05);        // z 방향으로 5 cm
```

---

## 2. 가장 자주 사용하는 함수

### 상태 읽기

| 함수 | 의미 |
| --- | --- |
| `readJoint()` | 현재 J1~J4 각도를 읽어 Serial Monitor에 출력 |
| `readTCP()` | BASE 좌표 기준 현재 TCP의 `x, y, z`를 출력 |

```cpp
readJoint();
readTCP();
```

### Joint 제어

| 함수 | 의미 | 기본 시간 |
| --- | --- | ---: |
| `moveHome()` | 학생 기준 모든 Joint를 `0 degree` 자세로 이동 | 2.0 s |
| `moveJointAbs(j1,j2,j3,j4)` | 지정한 절대 Joint 각도로 이동 | 2.0 s |
| `moveJointRel(dj1,dj2,dj3,dj4)` | 현재 각도에서 지정한 만큼 추가 이동 | 2.0 s |

**Absolute(절대 이동)** 는 목표 각도를 직접 지정합니다.

```cpp
moveJointAbs(0, 30, -20, 10);
```

**Relative(상대 이동)** 는 현재 자세에서 얼마만큼 더 움직일지 지정합니다.

```cpp
moveJointRel(0, 10, 0, 0);  // J2만 현재 위치에서 +10 degree
```

### TCP 제어

| 함수 | 의미 | 기본 시간 |
| --- | --- | ---: |
| `moveTCPAbs(x,y,z)` | BASE 기준 목표 TCP 좌표로 직선 이동 | 2.0 s |
| `moveTCPRel(dx,dy,dz)` | 현재 TCP에서 지정한 거리만큼 직선 이동 | 2.0 s |

```cpp
moveTCPAbs(0.18, 0.00, 0.12);
moveTCPRel(0.00, 0.00, 0.03);
```

`moveTCPRel(0, 0, 0.03)`은 현재 TCP에서 **z 방향으로 3 cm** 이동한다는 뜻입니다.

### Pitch / Gripper

| 함수 | 의미 |
| --- | --- |
| `keepHorizontal()` | 그리퍼를 바닥 기준 수평에 가깝게 맞춤 |
| `setPitch(pitch)` | 그리퍼의 절대 pitch를 degree로 지정 |
| `setGripper(true)` | 그리퍼 열기 |
| `setGripper(false)` | 그리퍼 닫기 |
| `openGripper()` | 그리퍼 열기 |
| `closeGripper()` | 그리퍼 닫기 |

```cpp
keepHorizontal();
setPitch(-5);
openGripper();
closeGripper();
```

### 외부 LED / Switch 보드

| 함수 | 의미 |
| --- | --- |
| `setLEDs(value)` | LED 4개를 비트 값으로 제어 |
| `SW1()` ~ `SW4()` | 각 Switch의 현재 상태 읽기 |

LED 값은 다음처럼 생각하면 됩니다.

| LED | 값 |
| --- | ---: |
| LED1 | 1 |
| LED2 | 2 |
| LED3 | 4 |
| LED4 | 8 |

```cpp
setLEDs(8);   // LED4만 ON
setLEDs(10);  // LED2 + LED4 ON
```

Switch는 `if`문과 함께 사용할 수 있습니다.

```cpp
if (SW3() == HIGH)
{
  setLEDs(1);
}
else
{
  setLEDs(0);
}
```

---

## 3. TCP 이동은 직선(Linear) 방식입니다

Joint 이동은 각 관절이 목표 각도로 이동합니다.
TCP 이동은 **Cartesian 공간에서 현재 TCP와 목표 TCP 사이를 직선으로 이동**하도록 ROBOTIS task-space trajectory를 사용합니다.

```text
현재 TCP ●──────────────● 목표 TCP
             직선 경로
```

예를 들어:

```cpp
moveTCPRel(0.00, 0.00, 0.05);
```

을 실행하면 현재 자세에서 TCP가 z 방향으로 5 cm 직선 이동합니다.

TCP의 `x, y, z` 위치만 바꾸는 함수에서는 현재 end-effector orientation을 유지합니다.

---

## 4. 동작 시간 `t` 사용법

모든 주요 이동 함수는 마지막에 동작 시간을 넣을 수 있습니다.

```cpp
moveJointAbs(0, 30, -20, 0, 3.0);
moveTCPRel(0, 0, 0.05, 3.0);
```

마지막 `3.0`은 **3초 정도에 걸쳐 동작하도록 요청**한다는 뜻입니다.

시간을 생략하면:

```cpp
moveJointAbs(0, 30, -20, 0);  // 기본 2초
moveTCPRel(0, 0, 0.05);       // 기본 2초
setPitch(10);                  // 기본 1.5초
```

처럼 기본값이 사용됩니다.

### 너무 빠른 명령은 자동으로 느려집니다

학생이 너무 짧은 시간을 입력해도 내부에서 이동 거리와 설정된 속도 기준을 계산하여 필요한 경우 동작 시간을 늘립니다.

```text
[SAFE] Move time adjusted to 4.00 sec.
```

같은 메시지가 나오면 오류가 아니라 **안전을 위해 시간이 자동 조정된 것**입니다.

> 실제 순간 속도 프로파일은 ROBOTIS trajectory가 생성하며, 이 프로젝트의 속도 설정은 최소 동작 시간을 계산하는 기준으로 사용됩니다.

---

## 5. Joint와 TCP 속도는 함께 조절됩니다

Joint는 `degree/s`, TCP는 `m/s`를 사용하기 때문에 두 속도를 같은 숫자로 맞추지는 않습니다.
대신 각각의 기준 속도에 **같은 비율(scale)** 을 적용합니다.

`omx_config.h`의 기본값:

```cpp
constexpr double OMX_MOTION_SPEED_SCALE = 1.0;
constexpr double OMX_MAX_JOINT_SPEED_DEG_S = 30.0;
constexpr double OMX_MAX_TCP_SPEED_M_S = 0.05;
```

현재 기준은:

```text
Joint : 30 degree/s
TCP   : 0.05 m/s = 5 cm/s
```

전체 로봇 동작을 더 천천히 하고 싶다면 운영자가 다음 값만 낮추면 됩니다.

```cpp
constexpr double OMX_MOTION_SPEED_SCALE = 0.5;
```

그러면 Joint와 TCP 모두 같은 비율로 느려집니다.

**학생은 보통 이 값을 수정할 필요가 없습니다.**

---

## 6. 안전 규칙

처음 로봇을 움직일 때는 다음 기준을 지켜주세요.

1. 처음에는 작은 각도와 작은 거리부터 테스트합니다.
2. Joint 명령은 기본적으로 `-90 ~ +90 degree` 범위에서 사용합니다.
3. 동작 시간은 가능하면 `1초 이상`으로 사용합니다.
4. 처음에는 로봇 주변을 비우고 Power OFF 또는 RESET을 바로 누를 수 있게 준비합니다.
5. 도달하기 어려운 TCP 좌표를 한 번에 크게 입력하지 않습니다.

안전 범위를 벗어나거나 IK 계산이 실패하면 Serial Monitor에 `[ERROR]` 메시지가 출력되고 해당 동작은 중단됩니다.

---

## 7. 따라 해보기

### 예제 1 — Joint와 TCP 확인

```cpp
void loop()
{
  moveHome();
  readJoint();
  readTCP();

  moveJointAbs(0, 20, -20, 0, 3.0);
  readJoint();
  readTCP();

  while (1);
}
```

### 예제 2 — 위로 3 cm 이동

```cpp
void loop()
{
  moveHome();
  moveTCPRel(0, 0, 0.03);

  while (1);
}
```

### 예제 3 — Switch로 동작 선택

```cpp
void loop()
{
  if (SW1() == HIGH)
  {
    moveTCPRel(0, 0, 0.01);
  }

  if (SW2() == HIGH)
  {
    moveHome();
  }
}
```

### 예제 4 — 간단한 Pick 동작

```cpp
void loop()
{
  moveHome();
  openGripper();

  moveTCPAbs(0.17, 0.10, 0.20);
  keepHorizontal();
  moveTCPRel(0, 0, -0.02);

  closeGripper();
  moveTCPRel(0, 0, 0.05);

  while (1);
}
```

실제 Pick 위치는 로봇과 물체 배치에 맞게 직접 확인해서 사용하세요.

---

## 8. Arduino 업로드 방법

1. `Omx_Manual.ino`를 Arduino IDE에서 엽니다.
2. Board를 `OpenCR Board`로 선택합니다.
3. OpenCR이 연결된 Port를 선택합니다.
4. 코드를 Upload 합니다.
5. Serial Monitor를 `115200 baud`로 엽니다.

정상 초기화되면 다음과 비슷한 메시지가 출력됩니다.

```text
[OK] OpenManipulator initialized.
==== OpenManipulator Started! ====
```

---

# 운영자 / 개발자 참고

아래 내용은 학생이 기본 실습을 할 때는 몰라도 됩니다.

## 9. 파일 구성

```text
.
├── Omx_Manual.ino   # 학생이 주로 수정
├── utils.h          # 학생용 public API
├── omx_internal.h   # 상태 동기화, trajectory, FK/IK, calibration 처리
├── omx_config.h     # 속도, 보정값, 외부 I/O 설정
├── AGENTS.md        # 저장소 개발 원칙
└── README.md
```

## 10. Software calibration

DYNAMIXEL Homing Offset을 필수로 사용하지 않고 software에서 joint zero를 보정합니다.

현재 설정:

```cpp
constexpr double OMX_JOINT_ZERO_OFFSET_RAD[4] = {
  0.0,
  -4.8 * OMX_DEG_TO_RAD,
  -4.8 * OMX_DEG_TO_RAD,
  0.0
};
```

좌표 관계:

```text
q_raw = q_student + offset
```

학생이:

```cpp
moveJointAbs(0, 0, 0, 0);
```

을 호출하면 실제 actuator에는 J2/J3 보정값이 자동으로 적용됩니다.

`readJoint()`, `readTCP()`, TCP FK/IK 역시 같은 보정 좌표계를 사용합니다.

`OMX_PITCH_CORRECTION_RAD = -0.14`는 joint zero와 별개의 end-effector 수평 보정값입니다. 로봇을 재조립하거나 교체했다면 실물에서 다시 확인해야 합니다.

## 11. 내부 제어 구조

```text
학생 API
   │
   ├─ Joint 명령
   │     ↓
   │  software calibration
   │     ↓
   │  실제 OpenManipulator / DYNAMIXEL
   │
   └─ TCP 명령
         ↓
      calibrated ROBOTIS model
      FK / IK / task trajectory
         ↓
      software calibration
         ↓
      실제 OpenManipulator / DYNAMIXEL
```

ROBOTIS의 FK, IK, trajectory 기능을 최대한 그대로 사용하고 wrapper는 학생용 단위 변환, calibration, 상태 동기화, 안전 처리만 담당합니다.

`processOpenManipulator()`에는 부팅 이후 계속 증가하는 절대 시간을 전달합니다.

```cpp
omx.processOpenManipulator(millis() / 1000.0);
```

학생 코드에서는 이 함수를 직접 사용할 필요가 없습니다.

## 12. 주요 운영 설정

`omx_config.h`에서 관리합니다.

```cpp
OMX_MOTION_SPEED_SCALE
OMX_MAX_JOINT_SPEED_DEG_S
OMX_MAX_TCP_SPEED_M_S
OMX_STUDENT_JOINT_LIMIT_DEG
OMX_JOINT_ZERO_OFFSET_RAD
OMX_PITCH_CORRECTION_RAD
```

외부 I/O 보드 핀은 현재 다음 값을 사용합니다.

```text
LED_PIN = {60, 61, 62, 63}
SW_PIN  = {50, 51, 52, 53}
```

이 값은 **OpenCR 내장 LED/SW가 아니라 대회용 외부 I/O 보드용 GPIO**입니다.
