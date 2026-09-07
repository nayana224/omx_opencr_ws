# OpenManipulator-X OpenCR Student API

ROBOTIS OpenMANIPULATOR-X를 OpenCR에서 직접 제어하기 위한 교육/대회용 Arduino 코드입니다.
학생은 DYNAMIXEL 통신, radian 변환, FK/IK, control loop를 직접 작성하지 않고 한 줄 함수로 로봇팔을 제어할 수 있습니다.

## 파일 구성

```text
.
├── Omx_Manual.ino   # 학생이 주로 수정하는 파일
├── utils.h          # 학생용 API
├── omx_internal.h   # 상태 동기화, FK/IK, trajectory, calibration
├── omx_config.h     # 속도, 보정값, 외부 I/O 설정
├── AGENTS.md        # 저장소 개발 원칙
└── README.md
```

## 기본 사용 예

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

학생은 `processOpenManipulator()`를 직접 호출할 필요가 없습니다.

## 학생용 API

| 함수 | 설명 |
| --- | --- |
| `readJoint()` | 현재 J1~J4 각도 읽기 `[degree]` |
| `readTCP()` | BASE 기준 현재 TCP 좌표 읽기 `[m]` |
| `moveHome(t)` | 학생 기준 `[0,0,0,0] degree` 자세로 이동 |
| `moveJointAbs(j1,j2,j3,j4,t)` | Joint 절대 이동 `[degree]` |
| `moveJointRel(dj1,dj2,dj3,dj4,t)` | Joint 상대 이동 `[degree]` |
| `moveTCPAbs(x,y,z,t)` | TCP 절대 좌표로 직선 이동 `[m]` |
| `moveTCPRel(dx,dy,dz,t)` | 현재 TCP에서 상대 좌표만큼 직선 이동 `[m]` |
| `keepHorizontal(t)` | 그리퍼를 바닥 기준 수평에 가깝게 유지 |
| `setPitch(pitch,t)` | 그리퍼 절대 pitch 지정 `[degree]` |
| `setGripper(true/false)` | 그리퍼 열기/닫기 |
| `openGripper()` | 그리퍼 열기 |
| `closeGripper()` | 그리퍼 닫기 |
| `setLEDs(value)` | 외부 LED 4개 제어 |
| `SW1()` ~ `SW4()` | 외부 switch 상태 읽기 |

기본 이동 시간은 Joint/TCP 모두 `2.0 sec`, pitch는 `1.5 sec`입니다.

## TCP 이동은 Linear 방식

`moveTCPAbs()`와 `moveTCPRel()`은 ROBOTIS task-space trajectory와 IK를 사용합니다.

```text
현재 TCP
   │
   │  직선 경로
   ▼
목표 TCP
```

XYZ 위치만 이동하고 현재 end-effector orientation은 유지합니다.
따라서 학생이 TCP 좌표를 지정하면 중간 경로도 Cartesian 공간에서 직선으로 움직입니다.

```cpp
moveTCPRel(0.00, 0.00, 0.05);
```

위 코드는 현재 TCP에서 z 방향으로 5 cm 직선 이동합니다.

## Joint / TCP 공통 속도 스케일

Joint는 `degree/s`, TCP는 `m/s`이므로 두 속도를 같은 숫자로 만드는 것은 의미가 없습니다.
대신 각각의 안전 속도 상한에 **같은 비율(scale)** 을 적용합니다.

기본 설정은 `omx_config.h`에 있습니다.

```cpp
constexpr double OMX_MOTION_SPEED_SCALE = 1.0;
constexpr double OMX_MAX_JOINT_SPEED_DEG_S = 30.0;
constexpr double OMX_MAX_TCP_SPEED_M_S = 0.05;
```

따라서 기본 최대 수준은:

```text
Joint : 30 degree/s
TCP   : 0.05 m/s = 5 cm/s
```

대회 전체 로봇을 더 느리게 하고 싶으면 한 값만 바꾸면 됩니다.

```cpp
constexpr double OMX_MOTION_SPEED_SCALE = 0.5;
```

그러면:

```text
Joint : 최대 15 degree/s
TCP   : 최대 2.5 cm/s
```

처럼 Joint와 TCP가 동시에 같은 비율로 느려집니다.

### `t`가 너무 짧은 경우

기존 교육 코드와 호환되도록 마지막 인자 `t`는 유지합니다.

```cpp
moveJointAbs(0, 60, 0, 0, 2.0);
moveTCPRel(0.00, 0.00, 0.10, 2.0);
```

다만 `t`가 너무 짧아 안전 속도 상한을 넘게 되면 실제 동작 시간은 자동으로 늘어납니다.

예를 들어 TCP를 20 cm 이동시키면서 `t=1.0`을 줘도 5 cm/s 상한을 유지하려면 최소 4초가 필요하므로 wrapper가 4초로 조정합니다.
Serial Monitor에는 다음과 같이 표시됩니다.

```text
[SAFE] Move time adjusted to 4.00 sec.
```

즉 학생에게 `t`는 **원하는 최소 동작 시간**이고, 안전을 위해 더 길어질 수 있습니다.

## Software calibration

DYNAMIXEL Homing Offset을 필수로 사용하지 않습니다.
실물 로봇의 zero 오차는 software에서 보정합니다.

현재 설정:

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
q_raw = q_student + offset
```

따라서 학생이:

```cpp
moveJointAbs(0, 0, 0, 0);
```

을 호출하면 내부에서 J2/J3 보정값이 자동 적용됩니다.
`readJoint()`, `readTCP()`, TCP IK도 같은 보정 좌표를 사용합니다.

`OMX_PITCH_CORRECTION_RAD = -0.14`는 별도의 실물 수평 보정값이며, 로봇을 다시 조립하거나 교체하면 실제 장비에서 재확인해야 합니다.

## 안전 범위

학생 실습용 Joint 명령은 추가로 `-90 ~ +90 degree` 범위를 검사합니다.
그 뒤 ROBOTIS 내부 joint limit도 다시 검사합니다.

기본 안전 설정:

```cpp
constexpr double OMX_STUDENT_JOINT_LIMIT_DEG = 90.0;
constexpr double OMX_MIN_MOVE_SEC = 1.0;
```

도달할 수 없는 TCP 좌표나 안전 범위를 벗어나는 명령은 Serial Monitor에 오류를 출력하고 동작을 중단합니다.

## 외부 I/O 보드

현재 GPIO는 대회용 외부 LED/SW 보드를 위한 값입니다.

```cpp
LED_PIN = {60, 61, 62, 63}
SW_PIN  = {50, 51, 52, 53}
```

OpenCR 내장 LED/SW용 핀이 아니므로 임의로 변경하지 않습니다.

## Arduino 업로드

1. `Omx_Manual.ino`를 엽니다.
2. Board에서 `OpenCR Board`를 선택합니다.
3. OpenCR 포트를 선택합니다.
4. 업로드합니다.
5. Serial Monitor를 `115200 baud`로 엽니다.

정상 초기화 시:

```text
[OK] OpenManipulator initialized.
==== OpenManipulator Started! ====
```

와 비슷한 메시지가 출력됩니다.

## 처음 실물에서 확인할 순서

```cpp
moveHome();
readJoint();
readTCP();

moveJointAbs(0, 10, -10, 0, 3.0);
moveTCPRel(0.00, 0.00, 0.01, 3.0);

openGripper();
closeGripper();
```

먼저 작은 Joint/TCP 이동으로 방향과 속도를 확인한 뒤 이동량을 늘리세요.
