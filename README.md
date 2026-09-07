# OpenMANIPULATOR-X OpenCR API

ROBOTIS **OpenMANIPULATOR-X**를 OpenCR + Arduino에서 간단한 함수 호출로 제어하기 위한 교육/대회용 코드입니다.

DYNAMIXEL 통신, radian 변환, FK/IK, `processOpenManipulator()` 같은 내부 처리는 wrapper에서 담당합니다.
사용자는 `Omx_Manual.ino`의 `loop()`에서 필요한 API를 호출하면 됩니다.

---

## Setup

### 수정할 파일

```text
Omx_Manual.ino
```

`setup()`은 기본 초기화가 포함되어 있으므로 일반적으로 `loop()`만 작성합니다.

### 단위

| 항목 | 단위 |
| --- | --- |
| Joint 각도 | `degree` |
| TCP 위치 | `meter` |
| 동작 시간 `t` | `second` |

예:

```cpp
moveJointAbs(0, 30, -20, 0);  // degree
moveTCPRel(0, 0, 0.05);       // z 방향 5 cm
```

### Arduino 업로드

1. Arduino IDE에서 `Omx_Manual.ino`를 엽니다.
2. Board를 `OpenCR Board`로 선택합니다.
3. OpenCR이 연결된 Port를 선택합니다.
4. Upload 합니다.
5. Serial Monitor를 `115200 baud`로 엽니다.

정상 초기화 시 다음과 비슷한 메시지가 출력됩니다.

```text
[OK] OpenManipulator initialized.
==== OpenManipulator Started! ====
```

---

## Quick Start

```cpp
void loop()
{
  moveHome();
  moveJointAbs(0, 20, -20, 0);
  moveTCPRel(0.00, 0.00, 0.03);
  closeGripper();

  while (1);  // 단일 시퀀스 종료
}
```

별도의 control loop 함수는 직접 호출할 필요가 없습니다.

---

## API

### Read

| 함수 | 설명 |
| --- | --- |
| `readJoint()` | 현재 J1~J4 각도를 Serial Monitor에 출력 `[degree]` |
| `readTCP()` | BASE 기준 현재 TCP의 `x, y, z`를 출력 `[m]` |

```cpp
readJoint();
readTCP();
```

### Joint

| 함수 | 설명 | 기본 시간 |
| --- | --- | ---: |
| `moveHome()` | 모든 Joint를 학생 기준 `0 degree` 자세로 이동 | 2.0 s |
| `moveJointAbs(j1,j2,j3,j4)` | 지정한 절대 Joint 각도로 이동 | 2.0 s |
| `moveJointRel(dj1,dj2,dj3,dj4)` | 현재 자세에서 지정한 각도만큼 상대 이동 | 2.0 s |

**Absolute**는 목표 각도를 직접 지정합니다.

```cpp
moveJointAbs(0, 30, -20, 10);
```

**Relative**는 현재 자세를 기준으로 변화량을 지정합니다.

```cpp
moveJointRel(0, 10, 0, 0);  // J2 +10 degree
```

### TCP

| 함수 | 설명 | 기본 시간 |
| --- | --- | ---: |
| `moveTCPAbs(x,y,z)` | BASE 기준 목표 TCP 좌표로 직선 이동 | 2.0 s |
| `moveTCPRel(dx,dy,dz)` | 현재 TCP에서 지정한 거리만큼 직선 이동 | 2.0 s |

```cpp
moveTCPAbs(0.18, 0.00, 0.12);
moveTCPRel(0.00, 0.00, 0.03);
```

`moveTCPRel(0, 0, 0.03)`은 현재 TCP에서 z 방향으로 **3 cm** 이동합니다.

### Pitch / Gripper

| 함수 | 설명 |
| --- | --- |
| `keepHorizontal()` | 그리퍼를 바닥 기준 수평에 가깝게 맞춤 |
| `setPitch(pitch)` | 그리퍼 절대 pitch 지정 `[degree]` |
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

`openGripper()` / `closeGripper()`는 기존 `setGripper(bool)`를 동일 기능의 명시적 API로 분리한 함수입니다.

### External I/O

| 함수 | 설명 |
| --- | --- |
| `setLEDs(value)` | 외부 LED 4개 제어 |
| `SW1()` ~ `SW4()` | 외부 Switch 상태 읽기 |

LED 값:

| LED | 값 |
| --- | ---: |
| LED1 | 1 |
| LED2 | 2 |
| LED3 | 4 |
| LED4 | 8 |

```cpp
setLEDs(8);   // LED4 ON
setLEDs(10);  // LED2 + LED4 ON
```

Switch는 일반적인 조건문과 함께 사용할 수 있습니다.

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

## Motion

### TCP Linear Motion

`moveTCPAbs()`와 `moveTCPRel()`은 ROBOTIS task-space trajectory와 IK를 사용합니다.
TCP의 현재 위치와 목표 위치 사이를 **Cartesian 공간의 직선 경로**로 이동합니다.

```text
현재 TCP ●──────────────● 목표 TCP
             Linear
```

```cpp
moveTCPRel(0.00, 0.00, 0.05);
```

위 명령은 현재 TCP에서 z 방향으로 5 cm 직선 이동합니다.
XYZ만 지정하는 TCP 이동에서는 현재 end-effector orientation을 유지합니다.

### Move Time

주요 이동 함수의 마지막 인자 `t`로 동작 시간을 지정할 수 있습니다.

```cpp
moveJointAbs(0, 30, -20, 0, 3.0);
moveTCPRel(0, 0, 0.05, 3.0);
```

시간을 생략하면 기본값이 적용됩니다.

```cpp
moveJointAbs(0, 30, -20, 0);  // 2.0 s
moveTCPRel(0, 0, 0.05);       // 2.0 s
setPitch(10);                  // 1.5 s
```

`t`가 너무 짧아 설정된 속도 기준을 넘는 경우 실제 동작 시간은 자동으로 늘어납니다.

```text
[SAFE] Move time adjusted to 4.00 sec.
```

### Speed Scale

Joint와 TCP는 단위가 다릅니다.

```text
Joint : degree/s
TCP   : m/s
```

따라서 숫자 속도를 같게 두지 않고, 각각의 기준 속도에 동일한 scale을 적용합니다.

기본 설정은 `src/omx/omx_config.h`에서 관리합니다.

```cpp
constexpr double OMX_MOTION_SPEED_SCALE = 1.0;
constexpr double OMX_MAX_JOINT_SPEED_DEG_S = 30.0;
constexpr double OMX_MAX_TCP_SPEED_M_S = 0.05;
```

기본 기준:

```text
Joint : 최대 30 degree/s
TCP   : 최대 0.05 m/s = 5 cm/s
```

전체 동작을 절반 수준으로 낮추려면:

```cpp
constexpr double OMX_MOTION_SPEED_SCALE = 0.5;
```

처럼 scale만 조정하면 됩니다.

---

## Safety

- Joint 명령은 기본적으로 `-90 ~ +90 degree` 범위에서 사용합니다.
- 동작 시간은 기본적으로 `1초 이상`이 되도록 제한합니다.
- 초기 테스트는 작은 Joint 각도와 작은 TCP 이동으로 수행합니다.
- 도달하기 어려운 TCP 좌표를 한 번에 크게 지정하지 않습니다.
- 동작 중에는 Power OFF 또는 RESET을 즉시 사용할 수 있도록 준비합니다.

안전 범위를 벗어나거나 IK 계산이 실패하면 Serial Monitor에 `[ERROR]`가 출력되고 해당 동작을 중단합니다.

---

## Examples

### Joint / TCP 확인

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

### TCP +3 cm

```cpp
void loop()
{
  moveHome();
  moveTCPRel(0, 0, 0.03);

  while (1);
}
```

### Switch 제어

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

### Pick 동작

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

실제 Pick 좌표는 로봇과 물체 배치에 맞게 확인해서 사용합니다.

---

## Developer Notes

### File Structure

```text
.
├── Omx_Manual.ino       # 사용자 코드
├── utils.h              # 학생용 public API
├── README.md
├── AGENTS.md            # 저장소 개발 원칙
└── src/
    └── omx/
        ├── omx_internal.h  # 상태 동기화, trajectory, FK/IK, calibration
        └── omx_config.h    # 속도, 보정값, 외부 I/O 설정
```

학생용 코드에서는 `utils.h`만 include합니다.
`src/omx/`는 runtime 및 설정을 분리하기 위한 내부 구현 디렉터리입니다.

### Software Calibration

DYNAMIXEL Homing Offset을 필수로 사용하지 않고 software에서 joint zero를 보정합니다.

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

따라서:

```cpp
moveJointAbs(0, 0, 0, 0);
```

을 호출하면 실제 actuator에는 J2/J3 zero 보정값이 자동 적용됩니다.

`readJoint()`, `readTCP()`, TCP FK/IK 역시 같은 보정 좌표계를 사용합니다.

`OMX_PITCH_CORRECTION_RAD = -0.14`는 joint zero와 별개의 end-effector 수평 보정값입니다. 로봇을 재조립하거나 교체했다면 실물에서 다시 확인해야 합니다.

### Internal Control

```text
Public API
   │
   ├─ Joint command
   │     ↓
   │  software calibration
   │     ↓
   │  OpenManipulator / DYNAMIXEL
   │
   └─ TCP command
         ↓
      calibrated ROBOTIS model
      FK / IK / task trajectory
         ↓
      software calibration
         ↓
      OpenManipulator / DYNAMIXEL
```

ROBOTIS의 FK, IK, trajectory 기능을 우선 사용하고 wrapper는 단위 변환, calibration, 상태 동기화, 안전 처리를 담당합니다.

`processOpenManipulator()`에는 부팅 이후 계속 증가하는 절대 시간을 전달합니다.

```cpp
omx.processOpenManipulator(millis() / 1000.0);
```

### Configuration

주요 설정은 `src/omx/omx_config.h`에서 관리합니다.

```text
OMX_MOTION_SPEED_SCALE
OMX_MAX_JOINT_SPEED_DEG_S
OMX_MAX_TCP_SPEED_M_S
OMX_STUDENT_JOINT_LIMIT_DEG
OMX_JOINT_ZERO_OFFSET_RAD
OMX_PITCH_CORRECTION_RAD
```

외부 I/O 보드 GPIO:

```text
LED : 60, 61, 62, 63
SW  : 50, 51, 52, 53
```

해당 핀은 OpenCR 내장 LED/SW가 아니라 대회용 외부 I/O 보드 기준입니다.
