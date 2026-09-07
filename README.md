# OpenManipulator-X OpenCR Student API

ROBOTIS OpenMANIPULATOR-X를 OpenCR에서 직접 제어하면서 C/C++ 기초 문법과 로봇팔 제어를 실습하기 위한 교육/대회용 Arduino 코드입니다.

학생은 DYNAMIXEL 통신, radian 변환, FK/IK 구현, OpenManipulator control loop를 직접 작성하지 않고 `moveJointAbs(...)`, `moveTCPRel(...)` 같은 한 줄 함수로 로봇을 움직일 수 있습니다.

내부에서는 가능한 한 ROBOTIS의 `OpenManipulator` / `RobotisManipulator` API와 trajectory, FK, IK 구현을 그대로 사용합니다.

## 파일 구성

```text
.
├── Omx_Manual.ino   # 학생이 주로 수정하는 Arduino 스케치
├── utils.h          # 학생용 public API
├── omx_internal.h   # 상태 동기화, trajectory, calibration 내부 처리
├── omx_config.h     # calibration / control / 외부 GPIO 설정
├── AGENTS.md        # 저장소 개발 원칙
└── README.md
```

학생 실습에서는 보통 `Omx_Manual.ino`와 아래 API 표만 보면 됩니다.

## 준비물

- ROBOTIS OpenMANIPULATOR-X
- OpenCR
- DYNAMIXEL 전원 및 통신 연결
- Arduino IDE 또는 Arduino CLI
- ROBOTIS OpenCR 보드 패키지
- `open_manipulator_libs`
- Eigen
- 대회용 외부 LED / Switch 보드(사용하는 경우)

## 업로드

1. Arduino IDE에서 `Omx_Manual.ino`를 엽니다.
2. Board를 `OpenCR Board`로 선택합니다.
3. OpenCR 포트를 선택합니다.
4. 업로드합니다.
5. Serial Monitor를 `115200 baud`로 엽니다.

정상적으로 joint feedback까지 읽으면 다음과 비슷한 메시지가 출력됩니다.

```text
[OK] OpenManipulator initialized and joint feedback synced.
==== OpenManipulator Started! ====
```

## 가장 간단한 사용 예

```cpp
void loop()
{
  moveHome();
  moveJointAbs(0, 20, -20, 0);
  moveTCPRel(0.00, 0.00, 0.03);
  closeGripper();

  while (1);
}
```

모든 motion 함수 안에서 필요한 ROBOTIS control loop가 실행되므로 학생이 `processOpenManipulator()`를 직접 호출할 필요가 없습니다.

## 학생용 API

### 상태 읽기

| 함수 | 반환값 | 설명 |
| --- | --- | --- |
| `readJoint()` | `std::vector<double>` | 현재 J1~J4를 보정된 degree 좌표로 출력/반환 |
| `readTCP()` | `Eigen::Vector3d` | 현재 gripper TCP의 `x, y, z`를 meter로 출력/반환 |

### Joint 제어

| 함수 | 단위 | 설명 |
| --- | --- | --- |
| `moveHome(t)` | sec | 학생 기준 `[0, 0, 0, 0] deg` 자세로 이동 |
| `moveJointAbs(j1, j2, j3, j4, t)` | degree, sec | 절대 joint 각도로 이동 |
| `moveJointRel(dj1, dj2, dj3, dj4, t)` | degree, sec | 현재 자세에서 상대 joint 각도만큼 이동 |

예:

```cpp
moveJointAbs(0, 30, -30, 0, 2.0);
moveJointRel(0, -10, 10, 0, 1.5);
```

### Cartesian / TCP 제어

| 함수 | 단위 | 설명 |
| --- | --- | --- |
| `moveTCPAbs(x, y, z, t)` | meter, sec | TCP를 절대 XYZ 위치로 이동 |
| `moveTCPRel(dx, dy, dz, t)` | meter, sec | 현재 TCP에서 상대 XYZ만큼 이동 |

`moveTCPAbs()`와 `moveTCPRel()`은 ROBOTIS의 task trajectory와 OpenMANIPULATOR-X IK를 사용합니다.

예:

```cpp
moveTCPAbs(0.20, 0.00, 0.10, 2.0);
moveTCPRel(0.00, 0.00, 0.03, 1.5);
```

### Pitch / Gripper

| 함수 | 설명 |
| --- | --- |
| `keepHorizontal(t)` | 현재 J1~J3를 유지하면서 end-effector를 수평에 가깝게 맞춤 |
| `setPitch(target_pitch_deg, t)` | 목표 pitch를 degree로 지정 |
| `setGripper(true)` | gripper 열기 |
| `setGripper(false)` | gripper 닫기 |
| `openGripper()` | gripper 열기 |
| `closeGripper()` | gripper 닫기 |

초급 학생에게는 `openGripper()` / `closeGripper()`가 직관적이고, 조건문 실습에서는 `setGripper(bool)`를 사용할 수 있습니다.

```cpp
if (SW1())
{
  setGripper(true);
}
```

### 외부 LED / Switch 보드

| 함수 | 설명 |
| --- | --- |
| `setLEDs(value)` | 하위 4비트로 외부 LED 4개 제어 |
| `SW1()` ~ `SW4()` | 외부 switch 상태 읽기 |

현재 GPIO는 대회용 외부 보드를 위한 설정입니다. OpenCR 내장 LED/SW 핀으로 변경하지 않습니다.

## Software calibration

이 프로젝트에서는 DYNAMIXEL Homing Offset을 필수로 사용하지 않습니다.

실물 로봇의 기구 조립/제로 오차를 software calibration으로 보정합니다.

현재 설정은 `omx_config.h`에 있습니다.

```cpp
constexpr double OMX_JOINT_ZERO_OFFSET_RAD[4] = {
  0.0,
  -4.8 * OMX_DEG_TO_RAD,
  -4.8 * OMX_DEG_TO_RAD,
  0.0
};
```

좌표 관계는 다음과 같습니다.

```text
actual actuator(raw) = student/calibrated joint + zero offset
```

따라서 학생이 다음을 호출하면:

```cpp
moveJointAbs(0, 0, 0, 0);
```

내부에서는 실물 로봇의 오차를 보정한 actuator 목표가 자동으로 생성됩니다.

### 왜 `moveJointAbs()`에 단순히 offset만 더하지 않는가?

Joint 명령에만 offset을 더하면 ROBOTIS FK/IK가 보는 joint 좌표와 학생이 보는 joint 좌표가 달라집니다.

이 저장소는 두 개의 ROBOTIS `OpenManipulator` 객체를 사용합니다.

```text
학생 joint/TCP 좌표
        │
        ▼
calibrated OpenManipulator model
  ├─ ROBOTIS FK
  ├─ ROBOTIS IK
  └─ ROBOTIS task trajectory
        │
        ▼
software calibration 변환
        │
        ▼
actual OpenManipulator / DYNAMIXEL
```

즉 FK/IK를 새로 구현하는 것이 아니라 ROBOTIS 모델을 하나 더 사용해 calibration 좌표계를 일관되게 유지합니다.

### Pitch correction

`OMX_PITCH_CORRECTION_RAD`는 joint zero offset과 별개의 실물 end-effector 자세 보정값입니다.

```cpp
constexpr double OMX_PITCH_CORRECTION_RAD = -0.14;
```

`keepHorizontal()`과 `setPitch()`에서만 사용합니다.

로봇을 교체하거나 기구를 다시 조립했다면 이 값과 joint offset은 실제 장비를 보고 다시 확인해야 합니다.

## ROBOTIS control loop 처리

ROBOTIS `processOpenManipulator()`에는 부팅 이후 계속 증가하는 시간이 전달되어야 합니다.

내부 구현은 다음 방식으로 동작합니다.

```cpp
omx.processOpenManipulator(millis() / 1000.0);
```

학생은 이를 직접 작성하지 않습니다.

각 motion 함수는 명령 직전에 trajectory clock과 현재 joint feedback을 갱신합니다. 따라서 명령 사이에 `delay()`가 있어도 다음 trajectory가 이전 시간값 때문에 중간부터 시작하지 않도록 구성했습니다.

## 정기구학 실습

```cpp
moveHome();
readTCP();

moveJointAbs(0, 20, -20, 0, 2.0);
readJoint();
readTCP();
```

관찰 포인트:

```text
Joint angle 변경
        ↓
ROBOTIS Forward Kinematics
        ↓
TCP XYZ 변경
```

## 역기구학 실습

```cpp
moveHome();
moveTCPAbs(0.18, 0.02, 0.10, 2.0);
readJoint();
readTCP();
```

관찰 포인트:

```text
TCP XYZ 목표
      ↓
ROBOTIS Inverse Kinematics
      ↓
J1~J4 목표 생성
```

## 단위

- Joint 입력/출력: degree
- TCP: meter
- 이동 시간: second
- ROBOTIS 내부 계산: radian

학생 API 경계에서 자동 변환합니다.

## 안전 주의

- 처음에는 넓은 공간에서 긴 이동시간으로 테스트하세요.
- TCP 좌표는 작은 값부터 변경하세요.
- 도달할 수 없는 TCP 좌표를 입력하면 Serial에 IK/workspace 오류가 출력될 수 있습니다.
- software calibration 때문에 실제 actuator limit과 학생 좌표 limit에는 약간의 차이가 있을 수 있으며, 내부에서 raw joint limit을 다시 검사합니다.
- 그리퍼 주변과 케이블 간섭을 확인하세요.

## 개발 원칙

저장소를 수정할 때는 `AGENTS.md`를 먼저 확인하세요.

핵심 원칙은 다음과 같습니다.

- 학생 API는 간단하게 유지
- calibration은 한 곳에서 관리
- FK/IK/trajectory는 ROBOTIS 구현 우선
- 내부 control loop를 학생에게 노출하지 않음
- 외부 LED/SW GPIO 설정 유지
