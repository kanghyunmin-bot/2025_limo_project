#!/bin/bash

echo "🚗 Path Follower v2.7 - 의존성 설치"
echo "===================================="

# 색상
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

# ROS2 확인
if [ ! -f "/opt/ros/humble/setup.bash" ]; then
    echo "❌ ROS2 Humble이 설치되지 않았습니다."
    exit 1
fi

# 의존성 설치
echo -e "${YELLOW}📦 시스템 의존성 설치 중...${NC}"
sudo apt update -qq
sudo apt install -y -qq \
    ros-humble-ackermann-msgs \
    python3-numpy \
    python3-scipy \
    python3-tk

echo -e "${GREEN}✅ 의존성 설치 완료!${NC}"
