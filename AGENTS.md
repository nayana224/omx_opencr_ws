# AGENTS.md

## Project purpose

이 저장소는 OpenCR에서 ROBOTIS OpenMANIPULATOR-X를 제어하는 교육/대회용 Arduino API를 제공합니다.
학생은 DYNAMIXEL 통신, radian 변환, FK/IK 구현, `processOpenManipulator()` 호출을 직접 다루지 않고 한 줄 함수 호출로 기본 로봇팔 제어를 수행할 수 있어야 합니다.

## Repository workflow

- 기본 작업 브랜치는 `main`입니다.
- 사용자가 별도로 요청하지 않는 한 새 브랜치나 PR을 만들지 않습니다.
- 새로운 작업을 시작할 때 이 `AGENTS.md`를 현재 설계 원칙에 맞게 먼저 갱신합니다.

## Core design rules

- 학생용 public API는 한 줄 호출을 우선합니다.
- Joint 입력은 degree, Cartesian 위치는 meter, 시간은 second를 기본 단위로 사용합니다.
- FK, IK, trajectory 생성은 가능한 한 ROBOTIS `OpenManipulator` / `RobotisManipulator` API를 사용합니다.
- ROBOTIS 기능을 별도로 재구현하지 않고 wrapper는 단위 변환, calibration, 상태 동기화, 안전 검사와 사용성 개선에 집중합니다.
- `processOpenManipulator()`에는 부팅 이후 계속 증가하는 절대 시간을 전달합니다.
- 학생용 API에서 DYNAMIXEL ID, raw actuator position, control-loop 호출을 노출하지 않습니다.
- 주석은 필요한 부분에만 짧고 명확한 한글로 작성합니다.

## README documentation rules

- README는 `Setup`, `Quick Start`, `API`, `Motion`, `Safety`, `Examples`, `Developer Notes`처럼 짧고 중립적인 섹션명을 우선합니다.
- 유아적이거나 지나치게 친절한 표현보다 기술 문서에 가까운 간결한 톤을 사용합니다.
- 첫 부분은 `Omx_Manual.ino`의 `loop()`를 바로 수정할 수 있도록 최소 사용법과 단위를 보여줍니다.
- 학생에게 필요한 사용법, 단위, API, 간단 예제, 안전 규칙을 먼저 배치합니다.
- calibration, shadow model, runtime 같은 내부 구현 설명은 학생용 사용법 뒤의 운영자/개발자 영역에 둡니다.
- 같은 내용을 여러 절에서 반복하지 않고 표와 짧은 예제를 우선합니다.
- Absolute/Relative, Joint/TCP 등 처음 접할 수 있는 용어는 한 문장으로 의미를 설명합니다.

## Motion invariants

- TCP 절대/상대 이동은 task-space trajectory를 사용해 직선 경로로 이동합니다.
- TCP 이동 중에는 현재 end-effector orientation을 유지합니다.
- Joint와 TCP는 동일한 global speed scale을 사용합니다.
- Joint와 TCP는 단위가 다르므로 숫자 속도를 같게 만들지 않고, 각각의 안전 속도 상한에 동일한 비율을 적용합니다.
- 사용자가 너무 짧은 이동 시간을 주면 wrapper가 안전 속도 기준을 넘지 않도록 실제 동작 시간을 자동으로 늘립니다.
- 기본 속도와 scale은 `omx_config.h` 한 곳에서 관리합니다.

## Calibration invariant

이 로봇은 기구 조립/제로 오차 때문에 software joint offset을 사용할 수 있습니다.

- calibration 값은 `omx_config.h` 한 곳에서 관리합니다.
- 개별 public motion 함수에 임의의 magic offset을 추가하지 않습니다.
- 학생이 사용하는 calibrated joint 좌표와 actuator가 사용하는 raw joint 좌표의 변환은 내부 helper를 통해서만 수행합니다.
- FK/IK와 TCP 읽기도 동일한 calibrated joint 좌표를 기준으로 계산합니다.
- DYNAMIXEL Homing Offset을 필수 전제로 만들지 않습니다.

## Public API invariants

기존 학생 코드의 다음 호출 형태는 특별한 이유 없이 깨뜨리지 않습니다.

```cpp
moveHome();
moveJointAbs(0, 20, -20, 0);
moveJointRel(0, 10, 0, 0);
moveTCPAbs(0.20, 0.00, 0.10);
moveTCPRel(0.00, 0.00, 0.03);
keepHorizontal();
setPitch(15);
setGripper(true);
readJoint();
readTCP();
```

필요한 새 API는 추가할 수 있지만 동일 동작의 alias를 불필요하게 늘리지 않습니다.

## Hardware I/O

- `LED_PIN`과 `SW_PIN`은 OpenCR 내장 LED/SW가 아니라 대회용 외부 보드 GPIO입니다.
- 핀 번호를 OpenCR 내장 LED/SW 번호로 임의 변경하지 않습니다.

## Safety and scope

- 학생 실습 기준 joint 명령 범위는 기본적으로 `-90 ~ +90 deg`를 사용합니다.
- ROBOTIS 내부 joint limit도 함께 확인합니다.
- IK 실패와 상태 읽기 실패는 학생이 이해할 수 있는 `Serial` 메시지로 표시합니다.
- 통신/상태 읽기 실패 시 0 값을 정상 상태처럼 사용해 후속 motion을 만들지 않습니다.
- 안전 관련 변경은 학생 코드 복잡도를 증가시키기보다 내부 wrapper에서 처리합니다.
