# 🚗 Path Follower v2.7.7

**ROS 2 Humble 기반 자율 경로 추종 시스템 (Pure Pursuit / Stanley Method)**

---

## 📋 빠른 시작 (새 환경)

### 1️⃣ 의존성 설치
```bash
cd ~/path_follower
./install_deps.sh
```

## 🔧 의존성

### ROS 2 패키지
- `ros-humble-ackermann-msgs`

### Python 패키지
- `python3-numpy`
- `python3-scipy`
- `python3-tk`

### 수동 설치
```bash
sudo apt install -y   ros-humble-ackermann-msgs   python3-numpy   python3-scipy   python3-tk
```

---

## 📦 파일 구조
```text
path_follower/
├── src/
│   └── path_follower_pkg/
│       ├── path_follower_pkg/              # Python 패키지
│       │   ├── __init__.py
│       │   ├── follower_node.py
│       │   ├── path_controller.py          # Pure Pursuit
│       │   ├── stanley_controller.py       # Stanley Method
│       │   ├── path_manager.py
│       │   ├── planner_interface.py
│       │   ├── velocity_profile.py
│       │   ├── fake_robot.py
│       │   ├── math_utils.py
│       │   ├── spline_utils.py
│       │   └── control_panel/              # GUI 모듈
│       │       ├── __init__.py
│       │       ├── node.py
│       │       ├── gui.py
│       │       └── main.py
│       ├── launch/
│       │   └── path_follower.launch.py
│       ├── resource/
│       │   └── path_follower_pkg
│       ├── setup.py                        # ✅ 자동 패키지 탐색
│       ├── setup.cfg
│       └── package.xml                     # ✅ 의존성 명시
├── rviz_config/
│   └── path_follower.rviz
├── install_deps.sh                         # ✅ 의존성 자동 설치
├── build.sh                                # ✅ 간편 빌드
└── README.md
```

---

## 🚀 실행 방법

### 한 번에 실행 (Tilix 필요)
`tilix`가 설치되어 있고 `colcon build`를 완료했다면, 아래 스크립트 하나로 네 개 터미널을 자동으로 띄울 수 있습니다.
```bash
./scripts/run_path_follower_tilix.sh
```

### 터미널 1: Static TF
```bash
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 map odom
```

### 터미널 2: Fake Robot
```bash
source install/setup.bash
ros2 run path_follower_pkg fake_robot
```

### 터미널 3: Path Follower + GUI
```bash
source install/setup.bash
ros2 launch path_follower_pkg path_follower.launch.py
```

### 터미널 4: RViz
```bash
rviz2 -d rviz_config/path_follower.rviz
```

---

## 🧭 플래너 모드 및 경로 잠금
- **플래너 선택(가로 라디오 버튼)**: GUI에서 `APF → A* → RRT → Dijkstra` 순으로 한 줄에 배치된 라디오 버튼을 눌러 전역 플래너를 즉시 전환합니다. 선택된 모드는 상태 배너에 바로 반영됩니다.
- **경로 잠금(global_locked / rrt_path_generated)**: 한 번 생성된 전역 경로는 플래그가 켜진 상태에서 다시 계산하지 않고 재사용합니다. 새 웨이포인트나 외부 경로를 받을 때만 잠금을 풀어 재계획하며, 코스트맵·제약 갱신은 **보간/제어점 굽힘만 업데이트**합니다.
- **플래너별 동작**
  - `RRT`: 코스트맵/제약을 (중심, 반경) 튜플로 전달해 벽을 뚫지 않고 우회 경로를 찾은 뒤 Bézier/스플라인으로 스무딩합니다.
  - `APF`: GUI에 있는 APF 파라미터 패널(step, attract/repel gain, 영향 거리, 목표/정체 허용치)에서 값을 입력해 적용하면 잠재장으로 우회 경로를 만듭니다.
  - `Ackermann`: 차량 기구학에 맞춘 곡선으로 웨이포인트를 잇되 장애물은 무시합니다(테스트·데모용).
  - `None/직선/스플라인`: 단순 연결 후 보간만 수행합니다.
- **글로벌 vs 로컬 갱신 주기 분리**: 글로벌 경로는 잠금 상태에서 고정 샘플 밀도로 유지하고, `local_bezier` 모드에서만 로봇 근처 세그먼트를 세밀히 다시 굽혀 동적 회피를 적용합니다.

---

## 🛡️ 로컬 Bézier + LiDAR 동적 회피 사용법
`local_bezier` 보간 모드와 LiDAR 제약(cp) 포인트를 함께 쓰면 글로벌 경로를 기준으로 로컬 경로를 실시간으로 굽혀서 장애물을 피할 수 있습니다. 아래 절차대로 설정하세요. (제약/코스트맵 기반 굽힘은 **`local_bezier`**에서 로컬 세그먼트에 적용되고, **`only_global_bezier`**에서는 글로벌 경로 전체를 Bézier 체인으로 만들 때 코스트맵 제약/장애물을 고려한 뒤 로컬 굽힘 없이 그대로 따라갑니다. 그 밖의 보간 모드는 이름대로 직선·서브샘플·스플라인/Bezier를 그대로 따릅니다.)

1. **동적 회피 파라미터 확인**
   - `enable_dynamic_avoidance`(기본 `True`)를 끄지 않았는지 확인합니다.
   - 2D LiDAR 스캔을 쓴다면 기본 토픽 `/scan`과 `lidar_message_type=scan`(기본값)을 유지합니다. 포인트 클라우드를 직접 쓸 때는 `lidar_message_type=pointcloud`와 토픽을 맞춰주세요.
   - Nav2 글로벌 코스트맵(`/global_costmap/costmap`, `OccupancyGrid`)을 같이 쓰면, 코스트맵 상 고비용 셀을 **로봇 주변(≈3m)·경로 인근(≈0.8m)** 에서 제약점으로 자동 추출해 Bézier를 밀어냅니다. 필요 시 토픽과 임계값을 `global_costmap_topic`, `global_cost_threshold` 등 파라미터로 조정하세요.
   - Nav2 코스트맵 셀을 **로봇 반경 + 여유(기본 0.20m + 0.05m)** 만큼 부풀린 원으로 바꿔 글로벌 Bézier 체인이 겹치는지 검사하고, 겹치면 중간 제어점(P1/P2)만 반복적으로 밀어냅니다. 부풀림 크기는 `global_costmap_robot_radius`, `global_costmap_safety_margin`, `global_costmap_inflate_margin` 파라미터로 맞출 수 있습니다.
   - 코스트맵이 빠르게 들어올 때 경로 재계산이 지연되지 않도록, 경로 생성에 쓰는 코스트 셀·경로 점을 부분 샘플링(`global_costmap_path_stride`)하고 재계산 주기를 스로틀(`global_costmap_replan_interval`, 기본 0.8초)합니다.
   - **코스트맵이 글로벌 경로와 겹치지 않으면** 기존 글로벌 경로/제약을 그대로 유지해 불필요한 재계산으로 곡선이 흔들리지 않도록 했습니다.
   - 글로벌 경로가 한 번 확정되면 기본적으로 고정(샘플 밀도도 일반 모드에서 2배 성기게)하고, `local_bezier`일 때만 세밀 샘플과 로컬 재계산을 적용해 글로벌·로컬 업데이트 주기를 분리했습니다.
   - 글로벌 경로 자체도 코스트맵을 기준으로 **직각 모서리를 둥글게** 피팅합니다. Nav2의 Dijkstra/Theta* 결과를 받아 Bézier 체인으로 매끈하게 만들고, 고비용 셀과 겹치는 세그먼트는 중간 제어점만 밀어내 복도/모서리에서 부드럽게 돌아갑니다.
   - 경로 생성 속도를 높이기 위해, 코스트맵에서 추출한 제약/장애물이 **해당 세그먼트와 겹치는 구간에서만** Bézier 제어점을 밀어냅니다. 세그먼트별로 가까운 장애물만 최대 32개까지 사용하고, 나머지는 원래 곡선 형태를 유지해 클릭 후 경로가 즉시 생성되도록 최적화했습니다.
   - 추가로, 글로벌 경로의 바운딩 박스(여유 0.6m+) 밖에 있는 코스트맵 장애물은 한 번에 걷어내어 세그먼트 반복 연산량을 줄이고 클릭 후 응답 속도를 높였습니다.
   - 제어점 밀어내기는 국민대 이수원 교수님의 **FMCL 베지어 충돌 검출 방식(De Casteljau + AABB 프루닝)** 을 참고해 계산량을 줄였습니다. 점 제약과 코스트맵 장애물을 모두 원형으로 통합한 뒤 충돌 구간에서만 P1/P2를 한 번에 밀어내어 저사양 PC에서도 빠르게 경로가 그려집니다.
   - 글로벌/로컬 Bézier를 만들기 전, Nav2 코스트맵을 원형 장애물 집합으로 단순화한 뒤 세그먼트 인근 장애물만 사용해 **얇은 RRT 경로**를 먼저 찾습니다. 이 RRT 결과를 그대로 Bézier 체인으로 피팅해 직선-곡선 혼합 구간만 다듬으므로, 3cm 간격의 과도한 샘플 없이도 충돌을 피할 수 있습니다.
   - 로컬 Bézier 샘플링도 3cm 고정격자가 아니라 **드카스텔쥬 길이 분할** 방식으로 필요한 만큼만 점을 찍어 커브가 꼬이지 않게 유지합니다.

2. **로컬 Bézier 모드 켜기**
```bash
ros2 topic pub --once /path_follower/interpolation_method std_msgs/String "data: 'local_bezier'"
```
웨이포인트는 스플라인 대신 **연속된 3차 Bézier 체인**으로 글로벌 경로를 만들고, 로봇 근처(약 0.5m) 구간만 LiDAR 제약을 반영해 다시 굽혀집니다.

글로벌 Bézier만 쓰고 싶다면 (코스트맵 제약을 반영한 전역 곡선, 로컬 굽힘 없음):
```bash
ros2 topic pub --once /path_follower/interpolation_method std_msgs/String "data: 'only_global_bezier'"
```
전역 경로를 Bézier 체인으로 매끈하게 만든 뒤 고정하고, 로컬 제약 기반 굽힘은 적용하지 않습니다.

3. **동작 확인**
   - LiDAR가 포인트 클라우드나 스캔을 내보내면 `odom` 프레임으로 변환된 제약점이 자동 반영돼 로컬 경로가 장애물을 피해 굽어집니다.
    - 글로벌 코스트맵이 들어오면, 코스트 셀이 많은 구간을 반경 기반 배제 영역으로 간주해 로봇 주변 경로 위에 제약점을 심고, LiDAR 제약과 합쳐서 한 번에 반영합니다. 코스트맵과 경로 프레임이 다르면 같은 평면이라고 가정해 경고 후 계산합니다. (코스트맵이 경로와 겹치지 않으면 글로벌 경로는 그대로 유지됩니다.)
    - 2D 스캔 기반일 때는 스캔 포인트가 0.3m 이내로 뭉친 구간을 하나의 장애물 클러스터로 보고 대표점을 cp로 씁니다. 대표점은 장애물 위치에서 추가로 0.33m 외곽으로 띄워 곡선이 장애물 표면에 달라붙지 않게 합니다. 클러스터마다 최근접 대표점을 골라 **반경 약 0.6m의 배제 영역**을 만들고, Bézier 곡선이 이 영역과 겹치면 중간 제어점(P1/P2)을 완만하게 밀어내어 우회합니다.
   - 제약점이 경로 진행 방향 앞쪽에 있을 때 더 강하게 제어점을 밀어내며, 로봇과 0.5m 이내면 선형 속도를 자동으로 줄이고 0.25m 이내면 완전히 정지합니다.
   - Bézier 제어점은 P0(로봇 위치)와 P3(글로벌 경로 복귀점)는 고정하고, 중간 P1/P2만 장애물을 피하도록 잠시 글로벌 경로를 벗어났다가 다시 원래 경로로 복귀합니다. 겹침이 남아 있는 동안(=P3에 도달하기 전) 매번 곡선을 다시 검사하며 제어점을 계속 밀어내므로 배제 영역을 끝까지 유지합니다. 제어점은 장애물에 가까울수록 더 강하게 곡선을 끌어내도록 거리 비례 힘을 적용해 겹침을 빨리 풀어냅니다.
   - 제어점 밀어내기에는 **ph offset(phase push)** 성분을 추가해, 장애물 측면으로 밀어내는 기본 오프셋에 더해 진행 방향으로 약간 밀어 우회폭을 키웁니다. 덕분에 곡선이 장애물 외곽을 크게 돌아나가면서도 P3로 되돌아오기 전까지 배제 영역을 벗어나지 않습니다.
   - LiDAR 제약이 감지되면 로컬 Bézier 구간 길이를 최대 약 1.4m까지 유동적으로 늘려 더 부드럽게 돌아가며, 제약이 사라지면 다시 기본 길이(약 0.6m)로 돌아갑니다.
   - RViz에서 `/local_path`를 시각화하면 실시간 굴곡을 확인할 수 있습니다.
   - 정적 장애물 회피용 글로벌 경로와 함께 사용하면 정적·동적 회피가 모두 적용됩니다.

필요 시 `enable_dynamic_avoidance:=false`를 전달하면 LiDAR 제약을 끈 상태로 로컬 Bézier를 테스트할 수 있습니다.

---

## 🌐 다른 노트북에서 설치

### 방법 1: Git Clone
```bash
git clone https://github.com/kanghyunmin-bot/path_follower.git
cd path_follower
./install_deps.sh
./build.sh
source install/setup.bash
```

### 방법 2: src만 복사
```bash
# USB/네트워크로 src 폴더만 복사한 경우
cd ~/path_follower
./install_deps.sh
./build.sh
source install/setup.bash
```

---

## 💡 주요 기능
- ✅ **Pure Pursuit** / **Stanley Method** / **Stanley+Feedforward**
- ✅ **Differential** / **Ackermann** Drive
- ✅ **RViz Clicked Point** 경로 생성 (Map / Grid 클릭을 분리해 선택 가능)
- ✅ **Planner Path** 연동
- ✅ **곡률 기반 속도 제어**
- ✅ **실시간 GUI 제어**
  - GUI에서 Local/Global Bézier 제약 반경과 **Costmap 회피 거리(글로벌 Clearance)** 를 숫자로 입력해 즉시 반영
  - 입력한 글로벌 Clearance 값이 코스트맵 거리창에도 적용되어, 경로와 겹치지 않을 때는 불필요한 재계산 없이 안정적으로 유지
  - 글로벌/로컬 베지어 외에도 **RRT / A* / Dijkstra / APF(인공 잠재장)** 전역 플래너 모드를 GUI에서 선택 가능
  - **APF Parameters** 패널에서 step, attract/repel gain, influence distance, goal/stall tolerance를 입력 후 Apply 시 즉시 반영

### Path Source 선택 (GUI)
- **RViz Map Click**: Nav2 Publish Point의 `/clicked_point`(또는 `/clicked_point_map`)을 맵 프레임에서 사용하며, 코스트맵 제약을 적용해 장애물을 피해 Bézier를 굽힙니다.
- **RViz Grid Click**: Publish Point `/clicked_point` 또는 `/clicked_point_grid`를 사용하며, 코스트맵 제약을 무시하고 순수 그리드 클릭 기반 경로만 유지합니다.
- **Planner Path**: `/planner/path` 입력을 그대로 사용합니다.

---

## 🛑 감속·로그 정책
- **도착 감속**: `_calculate_curvatures_and_velocities`가 곡률 기반 속도에 더해 **목표점 1m 전부터 선형 감속**을 적용해 급정지를 피합니다(최소 0.15 m/s 유지).
- **경로 기록 제한**: `path_recorder`가 실제 주행 궤적을 5,000포인트로 제한해 장시간 실행 시 메모리·RViz 부담을 방지합니다.

---

## 📝 버전
- **v2.7.7** (2025-11-09): RViz 경로 소스를 **Map Click / Grid Click**으로 분리해 코스트맵 적용 여부를 명확히 하고, GUI에서 바로 선택·제어하도록 추가
- **v2.7.6** (2025-11-09): RRT/APF/Ackermann 브리지와 플래너 잠금을 통합해 경로 포맷 불일치(점↔원)와 반복 재계산 문제를 해결하고, 베지어 보간만 다시 적용하는 흐름으로 정리
- **v2.7.5** (2025-11-09): 글로벌 경로 잠금 플래그를 추가해 클릭 후 생성된 전역 Bézier를 유지하고, 제약·코스트맵 갱신 시에도 글로벌 경로를 덮어쓰지 않도록 정리
- **v2.7.4** (2025-11-09): 플래너 선택 라디오 버튼을 APF / A* / RRT / Dijkstra 가로 배치로 재구성하고, GUI 상태 배너와 연동
- **v2.7.3** (2025-11-09): APF 선택/파라미터 입력 패널을 GUI에 추가하고, Stanley+Feedforward 제어 모드를 alongside 유지된 Stanley/Pure Pursuit와 함께 선택 가능하게 구성
- **v2.7.2** (2025-11-09): LiDAR·코스트맵 제약 반경/글로벌 클리어런스 입력 박스를 GUI에 추가해 숫자 입력으로 반경을 전달하고, 플래너 모드 선택·전역 보간 모드를 추가
- **v2.7.1** (2025-11-09): LiDAR/코스트맵 제약 추출을 벡터화하고 브로드페이즈(볼록 껍질·AABB)로 충돌 검사를 줄인 뒤, 곡률 기반 속도에 목표 1m 전 감속과 경로 기록 5,000포인트 제한을 도입
- **v2.7.0** (2025-11-09): `setup.py` 자동화, 의존성 스크립트 추가

---

## 📄 라이센스
Apache-2.0 License
