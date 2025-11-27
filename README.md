# 🚗 Path Follower v2.7

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

## 🛡️ 로컬 Bézier + LiDAR 동적 회피 사용법
`local_bezier` 보간 모드와 LiDAR 제약(cp) 포인트를 함께 쓰면 글로벌 경로를 기준으로 로컬 경로를 실시간으로 굽혀서 장애물을 피할 수 있습니다. 아래 절차대로 설정하세요.

1. **동적 회피 파라미터 확인**
   - `enable_dynamic_avoidance`(기본 `True`)를 끄지 않았는지 확인합니다.
   - 2D LiDAR 스캔을 쓴다면 기본 토픽 `/scan`과 `lidar_message_type=scan`(기본값)을 유지합니다. 포인트 클라우드를 직접 쓸 때는 `lidar_message_type=pointcloud`와 토픽을 맞춰주세요.

2. **로컬 Bézier 모드 켜기**
   ```bash
   ros2 topic pub --once /path_follower/interpolation_method std_msgs/String "data: 'local_bezier'"
   ```
   글로벌 경로는 그대로 유지하면서 로봇 근처(약 0.5m) 구간만 Bézier로 재생성됩니다.

3. **동작 확인**
   - LiDAR가 포인트 클라우드나 스캔을 내보내면 `odom` 프레임으로 변환된 제약점이 자동 반영돼 로컬 경로가 장애물을 피해 굽어집니다.
   - 2D 스캔 기반일 때는 스캔 포인트가 0.3m 이내로 뭉친 구간을 하나의 장애물 클러스터로 보고 대표점을 cp로 씁니다. 클러스터마다 최근접 대표점을 골라 경로가 겹치면 Bézier 중간 제어점(P1/P2)만 밀어내어 우회합니다.
   - 제약점이 경로 진행 방향 앞쪽에 있을 때 더 강하게 제어점을 밀어내며, 로봇과 0.5m 이내면 선형 속도를 자동으로 줄이고 0.25m 이내면 완전히 정지합니다.
   - Bézier 제어점은 P0(로봇 위치)와 P3(글로벌 경로 복귀점)는 고정하고, 중간 P1/P2만 장애물을 피하도록 잠시 글로벌 경로를 벗어났다가 다시 원래 경로로 복귀합니다. 제어점은 장애물 방향으로 붙지 않고 장애물에서 곡선을 끌어내도록 최근접 점을 기준으로 밀어냅니다.
   - LiDAR 제약이 감지되면 로컬 Bézier 구간 길이를 최대 약 1.0m까지 유동적으로 늘려 더 부드럽게 돌아가며, 제약이 사라지면 다시 기본 길이(약 0.5m)로 돌아갑니다.
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
- ✅ **Pure Pursuit** / **Stanley Method**
- ✅ **Differential** / **Ackermann** Drive
- ✅ **RViz Clicked Point** 경로 생성
- ✅ **Planner Path** 연동
- ✅ **곡률 기반 속도 제어**
- ✅ **실시간 GUI 제어**

---

## 📝 버전
- **v2.7.0** (2025-11-09): `setup.py` 자동화, 의존성 스크립트 추가

---

## 📄 라이센스
Apache-2.0 License
