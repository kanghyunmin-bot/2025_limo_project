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
