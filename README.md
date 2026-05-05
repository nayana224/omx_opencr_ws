# OpenManipulator-X OpenCR Manual Study Sketch

ROBOTIS OpenManipulator-X를 OpenCR 보드에서 직접 움직이며 정기구학, 역기구학, TCP 이동, 조인트 이동을 실습하기 위한 Arduino 스케치입니다. 복잡한 예제 코드를 매번 작성하지 않아도 `utils.h`에 정의된 사용자 함수만 호출해서 로봇팔의 자세와 위치 변화를 관찰할 수 있도록 구성했습니다.

## 구성

```text
.
├── Omx_Manual.ino   # Arduino 메인 스케치
├── utils.h          # OpenManipulator-X 제어용 사용자 함수 모음
└── README.md
```

## 준비물

- OpenManipulator-X
- OpenCR 보드
- DYNAMIXEL 전원 및 통신 연결
- Arduino IDE 또는 Arduino CLI
- ROBOTIS OpenCR 보드 패키지
- `open_manipulator_libs` 라이브러리
- `Eigen` 라이브러리

보드와 라이브러리 설치는 ROBOTIS OpenManipulator-X/OpenCR Arduino 환경 설정을 먼저 완료한 뒤 진행하세요.

## 업로드 방법

1. Arduino IDE에서 `Omx_Manual.ino`를 엽니다.
2. 보드를 `OpenCR Board`로 선택합니다.
3. OpenCR이 연결된 포트를 선택합니다.
4. 스케치를 업로드합니다.
5. 시리얼 모니터를 `115200 baud`로 열어 상태 메시지를 확인합니다.

업로드 후 기본 `loop()`는 비어 있습니다. 원하는 함수 예제를 `loop()` 안에서 하나씩 주석 해제하거나 직접 호출하면서 실습하면 됩니다.

## 기본 사용 예시

```cpp
void loop()
{
  moveHome();
  readJoint();
  readTCP();

  moveJointAbs(0, 20, -20, 0, 2.0);
  readTCP();

  moveTCPRel(0.00, 0.00, 0.03, 1.5);
  keepHorizontal();

  while (1);
}
```

반복 실행을 막고 싶다면 예시처럼 마지막에 `while (1);`을 넣어 한 번만 실행되도록 하세요.

## 사용자 함수

### 초기화와 내부 루프

| 함수 | 설명 |
| --- | --- |
| `initManipulator()` | OpenManipulator-X를 초기화합니다. `setup()`에서 호출됩니다. |
| `runManipulator(sec)` | 지정한 시간 동안 내부 제어 루프를 실행합니다. 이동 함수 내부에서 자동 호출됩니다. |
| `setPins()` | OpenCR의 LED/SW 핀을 설정합니다. |

### 상태 읽기

| 함수 | 반환값 | 설명 |
| --- | --- | --- |
| `readJoint()` | `std::vector<double>` | 현재 J1~J4 조인트 각도를 degree 단위로 출력하고 반환합니다. |
| `readTCP()` | `Eigen::Vector3d` | 현재 gripper TCP의 `x, y, z` 위치를 meter 단위로 출력하고 반환합니다. |

### 조인트 공간 이동

| 함수 | 단위 | 설명 |
| --- | --- | --- |
| `moveHome(t)` | sec | 기본 홈 자세로 이동합니다. |
| `moveJointAbs(j1, j2, j3, j4, t)` | degree, sec | J1~J4 목표 각도로 절대 이동합니다. |
| `moveJointRel(dj1, dj2, dj3, dj4, t)` | degree, sec | 현재 각도 기준으로 상대 이동합니다. |

예시:

```cpp
moveJointAbs(0, 30, -30, 0, 2.0);
moveJointRel(0, -10, 10, 0, 1.5);
```

### 작업 공간 이동

| 함수 | 단위 | 설명 |
| --- | --- | --- |
| `moveTCPAbs(x, y, z, t)` | meter, sec | gripper TCP를 목표 좌표로 이동합니다. OpenManipulator 라이브러리의 역기구학을 사용합니다. |
| `moveTCPRel(dx, dy, dz, t)` | meter, sec | 현재 TCP 위치 기준으로 상대 이동합니다. |

예시:

```cpp
moveTCPAbs(0.20, 0.00, 0.10, 2.0);
moveTCPRel(0.00, 0.03, 0.00, 1.5);
```

### 자세와 그리퍼

| 함수 | 설명 |
| --- | --- |
| `keepHorizontal(t)` | J2, J3, J4 관계를 이용해 TCP pitch가 수평에 가깝도록 보정합니다. |
| `setPitch(target_pitch_deg, t)` | 원하는 pitch 각도를 degree 단위로 지정합니다. |
| `setGripper(open, t)` | `true`면 열기, `false`면 닫기 명령을 보냅니다. |

### OpenCR LED/SW

| 함수 | 설명 |
| --- | --- |
| `setLEDs(value)` | 하위 4비트 값으로 LED 4개를 제어합니다. 예: `setLEDs(0b0101);` |
| `SW1()`, `SW2()`, `SW3()`, `SW4()` | OpenCR 스위치 입력값을 읽습니다. |

## 정기구학/역기구학 실습 흐름

### 정기구학 관찰

조인트 각도를 바꾼 뒤 TCP 좌표가 어떻게 변하는지 확인합니다.

```cpp
moveHome();
readTCP();

moveJointAbs(0, 20, -20, 0, 2.0);
readJoint();
readTCP();
```

### 역기구학 관찰

TCP 목표 좌표를 지정한 뒤 로봇이 어떤 조인트 각도로 이동했는지 확인합니다.

```cpp
moveHome();

moveTCPAbs(0.18, 0.02, 0.10, 2.0);
readTCP();
readJoint();
```

### 상대 이동 관찰

현재 위치에서 조금씩 이동시키며 좌표계 방향을 익힙니다.

```cpp
moveHome();
moveTCPRel(0.02, 0.00, 0.00, 1.5);
moveTCPRel(0.00, 0.02, 0.00, 1.5);
moveTCPRel(0.00, 0.00, 0.02, 1.5);
readTCP();
```

## 단위와 보정값

- 조인트 입력값: degree
- TCP 입력값: meter
- 이동 시간: second
- 내부 OpenManipulator 라이브러리 명령: radian 기반
- `moveJointAbs()`와 `moveHome()`은 J2/J3에 `-4.8 deg` 수동 보정값을 적용합니다.
- `keepHorizontal()`과 `setPitch()`는 J4 계산에 `-0.14 rad` pitch 보정값을 적용합니다.

로봇팔 조립 상태나 캘리브레이션에 따라 보정값은 달라질 수 있습니다. 실제 TCP가 수평에서 벗어나면 `utils.h`의 `OMX_JOINT_OFFSET_DEG`, `OMX_PITCH_OFFSET_RAD` 값을 조금씩 조정하세요.

## 안전 주의

- 처음 실습할 때는 항상 넓은 공간에서 낮은 속도, 긴 이동 시간으로 테스트하세요.
- `moveTCPAbs()`는 도달 불가능한 좌표를 넣으면 이동하지 않거나 예상과 다른 자세가 나올 수 있습니다.
- 전원 인가 직후 로봇팔 주변에 손이나 물체가 없는지 확인하세요.
- 좌표와 각도는 작은 값부터 바꾸며 관찰하세요.
- 그리퍼가 물체를 잡고 있을 때는 충돌과 케이블 꼬임을 확인하세요.

## 코드 점검 메모

- `utils.h`는 사용자가 바로 호출하기 쉬운 래퍼 함수 중심으로 잘 분리되어 있습니다.
- 각 이동 함수가 내부에서 `runManipulator()`를 호출하므로, 사용자는 별도의 제어 루프를 직접 작성하지 않아도 됩니다.
- 각도/좌표 단위를 함수 이름과 README에 명시해 실습 중 혼동을 줄였습니다.
- Arduino 코어와 이름이 겹칠 수 있는 단위 변환 상수에는 `OMX_` 접두어를 붙였습니다.
- OpenCR 보드/ROBOTIS 라이브러리가 필요한 프로젝트라 이 환경이 없는 PC에서는 일반 C++ 컴파일로 검증할 수 없습니다.
