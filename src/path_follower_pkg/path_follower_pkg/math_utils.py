#!/usr/bin/env python3
"""
수학 유틸리티 모듈
- 각도 정규화
- Quaternion 변환
"""
import math
from geometry_msgs.msg import Quaternion


def wrap_to_pi(a: float) -> float:
    """각도를 [-π, π] 범위로 정규화"""
    return (a + math.pi) % (2 * math.pi) - math.pi


def yaw_to_quat(yaw: float) -> Quaternion:
    """Yaw 각도를 Quaternion으로 변환"""
    half = 0.5 * yaw
    return Quaternion(x=0.0, y=0.0, z=math.sin(half), w=math.cos(half))
