# Path Follower 코드 기반 기술 검토

본 메모는 `path_follower_pkg` 소스 코드를 직접 검토해 정리한 내용이다. README 없이 코드 흐름을 따라가며 각 구성요소의 역할과 기대 성능, 잠재적 리스크를 기록했다.

## 전체 아키텍처
- `follower_node.py`에서 ROS2 노드가 초기화되며, **경로 생성/보간(PathManager)**, **제어기(Pure Pursuit/Stanley)**, **경로 기록 및 정확도 계산**을 포함한 풀 스택을 구성한다. 제어 루프는 60 Hz, 로컬 경로 업데이트는 15 Hz, 글로벌 경로 퍼블리시는 1 Hz로 분리되어 실행된다.【F:src/path_follower_pkg/path_follower_pkg/follower_node.py†L17-L119】
- 구동 모드(`drive_mode`)와 제어 모드(`control_mode`)는 런타임 토픽으로 바꿀 수 있으며, 각 모드 전환 시 내부 상태를 리셋한다.【F:src/path_follower_pkg/path_follower_pkg/follower_node.py†L52-L113】【F:src/path_follower_pkg/path_follower_pkg/stanley_controller.py†L34-L79】

## 경로 생성/보간 (`PathManager`)
- 외부 플래너 `/planner/path`나 RViz 클릭 포인트를 수집해 경로를 갱신한다. 보간 방식으로 spline, linear, subsample, (로컬) Bézier를 지원하며, 아커만 전용 곡선 계획기도 내장되어 있다.【F:src/path_follower_pkg/path_follower_pkg/path_manager.py†L23-L119】
- **속도 프로파일 분기**: 차동 모드에서는 구간 전체를 최대속도 `v_max`로 설정하고, 아커만 모드는 곡률을 계산해 `sqrt(a_lat/|κ|)` 기반 속도 프로파일을 적용한다.【F:src/path_follower_pkg/path_follower_pkg/path_manager.py†L80-L192】
- 로컬 Bézier 스플라이싱으로 로봇 주변 경로를 재구성할 수 있으나, `BEZIER_AVAILABLE` 플래그에 따라 모듈 존재 여부에 의존한다.【F:src/path_follower_pkg/path_follower_pkg/path_manager.py†L11-L17】【F:src/path_follower_pkg/path_follower_pkg/path_manager.py†L138-L170】

## 제어 로직
### Pure Pursuit (`PathController`)
- 최근접 포인트를 찾은 뒤 **속도 비례 룩어헤드**(0.3–1.5 m 범위)와 최종 포인트 요각을 모두 고려한 조향각을 계산한다. 조향은 ±35°로 클리핑되고, 각속도는 조향 기반 아커만 모델로 변환한 뒤 ±2.5 rad/s로 제한한다.【F:src/path_follower_pkg/path_follower_pkg/path_controller.py†L10-L52】【F:src/path_follower_pkg/path_follower_pkg/path_controller.py†L54-L73】

### Stanley (`StanleyController`)
- 헤딩·횡방향 오차를 계산한 후, **차동/아커만 별도 분기**로 제어를 생성한다.【F:src/path_follower_pkg/path_follower_pkg/stanley_controller.py†L19-L83】
  - 차동 모드: 헤딩 오차가 큰 경우 제자리 회전을 허용하고(속도 의존 임계값), 그 외에는 스탠리 조향각→곡률→각속도 변환에 헤딩 보정(0.3×)을 더한다.【F:src/path_follower_pkg/path_follower_pkg/stanley_controller.py†L38-L66】
  - 아커만 모드: 동일한 스탠리 조향각을 사용하되 제자리 회전 없이 아커만 기하(2*v*sin δ/L)로 각속도를 만든다.【F:src/path_follower_pkg/path_follower_pkg/stanley_controller.py†L68-L81】

## 성능 기대치 및 리스크 (코드 기반)
- **저속 실내/테스트 환경**: 60 Hz 제어와 15 Hz 로컬 업데이트, 경로 보간(스플라인/Bezier)이 결합되어 0.2–1.0 m/s 수준에서는 부드러운 추종이 기대된다. 차동 모드의 제자리 회전 분기가 큰 헤딩 오차 회복을 돕는다.【F:src/path_follower_pkg/path_follower_pkg/follower_node.py†L73-L115】【F:src/path_follower_pkg/path_follower_pkg/stanley_controller.py†L38-L66】
- **곡률 적응**: 아커만 모드에서만 곡률 기반 감속을 사용하므로, 차동 모드로 급커브를 고속 주행하면 횡슬립을 제어하는 수단이 각속도 클리핑뿐이다.【F:src/path_follower_pkg/path_follower_pkg/path_manager.py†L84-L192】【F:src/path_follower_pkg/path_follower_pkg/stanley_controller.py†L43-L66】
- **장애물/재경로 부재**: 입력 경로를 그대로 추종하며 로컬 장애물 회피나 동적 재경로 로직이 없다. 센서 융합 기반 슬립/헤딩 보정도 구현돼 있지 않아 고마찰 변동 환경에서는 오차가 커질 수 있다.【F:src/path_follower_pkg/path_follower_pkg/follower_node.py†L94-L169】【F:src/path_follower_pkg/path_follower_pkg/path_manager.py†L66-L106】
- **게인 하드코딩**: Stanley의 `k_e`, Pure Pursuit의 `k_theta`, 속도 클립 값 등이 코드 상수로 박혀 있어 차량별/속도별 자동 튜닝 기능은 없다.【F:src/path_follower_pkg/path_follower_pkg/path_controller.py†L10-L72】【F:src/path_follower_pkg/path_follower_pkg/stanley_controller.py†L16-L67】

## 시뮬레이션/실행 참고
- 제어 루프 주파수, 보간 방식, 구동 모드는 모두 토픽(`/path_follower/*`)으로 조정 가능하므로 실제 시뮬 돌릴 때는 모드를 전환해 응답 변화를 확인하는 것이 좋다. 장애물 회피가 없으므로 맵 상에 동적 객체가 없을 때 안정성이 평가된다.【F:src/path_follower_pkg/path_follower_pkg/follower_node.py†L52-L145】

