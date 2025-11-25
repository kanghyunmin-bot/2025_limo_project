#!/usr/bin/env python3
import math

def quaternion_to_yaw(q):
    return 2.0 * math.atan2(q.z, q.w)

def normalize_angle(angle):
    while angle > math.pi:
        angle -= 2 * math.pi
    while angle < -math.pi:
        angle += 2 * math.pi
    return angle

def find_nearest_point_on_path(robot_pos, robot_yaw, path_poses, last_idx=0):
    """✅ 균형잡힌 nearest point"""
    min_dist = float('inf')
    nearest_idx = last_idx
    
    # 1차: 주변 탐색
    search_start = max(0, last_idx - 5)
    search_end = min(len(path_poses), last_idx + 50)
    
    for i in range(search_start, search_end):
        px = path_poses[i].pose.position.x
        py = path_poses[i].pose.position.y
        dist = math.hypot(px - robot_pos[0], py - robot_pos[1])
        
        if dist < min_dist:
            min_dist = dist
            nearest_idx = i
    
    # 2차: fallback
    if min_dist > 1.0 or nearest_idx == last_idx:
        for i in range(0, len(path_poses)):
            px = path_poses[i].pose.position.x
            py = path_poses[i].pose.position.y
            dist = math.hypot(px - robot_pos[0], py - robot_pos[1])
            
            if dist < min_dist:
                min_dist = dist
                nearest_idx = i
    
    return nearest_idx, min_dist

def compute_cross_track_error(robot_pos, robot_yaw, path_pose):
    px = path_pose.pose.position.x
    py = path_pose.pose.position.y
    path_yaw = quaternion_to_yaw(path_pose.pose.orientation)
    
    dx = px - robot_pos[0]
    dy = py - robot_pos[1]
    
    cross_track_error = -math.sin(path_yaw) * dx + math.cos(path_yaw) * dy
    heading_error = normalize_angle(path_yaw - robot_yaw)
    
    return cross_track_error, heading_error
