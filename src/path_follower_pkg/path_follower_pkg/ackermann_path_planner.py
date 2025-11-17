#!/usr/bin/env python3
import math

def cubic_bezier(p0, p1, p2, p3, num_points=20):
    points = []
    for i in range(num_points+1):
        t = i / num_points
        x = (1 - t)**3 * p0[0] + 3 * (1 - t)**2 * t * p1[0] \
            + 3 * (1-t) * t**2 * p2[0] + t**3 * p3[0]
        y = (1 - t)**3 * p0[1] + 3 * (1 - t)**2 * t * p1[1] \
            + 3 * (1-t) * t**2 * p2[1] + t**3 * p3[1]
        points.append((x, y))
    return points

class AckermannPathPlanner:
    def __init__(self, control_dist_factor=0.3):
        self.control_dist_factor = control_dist_factor
    
    def plan_path(self, waypoints):
        if len(waypoints) < 2:
            return waypoints
        n = len(waypoints)
        path = []
        for i in range(n-1):
            p0 = waypoints[i]
            p3 = waypoints[i+1]
            
            # 방향 예측
            if i == 0:
                direction0 = math.atan2(p3[1] - p0[1], p3[0] - p0[0])
            else:
                prev = waypoints[i-1]
                direction0 = math.atan2(p3[1] - prev[1], p3[0] - prev[0]) / 2
            
            if i == n-2:
                direction3 = math.atan2(p3[1] - p0[1], p3[0] - p0[0])
            else:
                nxt = waypoints[i+2]
                direction3 = math.atan2(nxt[1] - p0[1], nxt[0] - p0[0]) / 2

            dist = math.hypot(p3[0]-p0[0], p3[1]-p0[1])
            d_control = dist * self.control_dist_factor
            
            p1 = (p0[0] + d_control * math.cos(direction0),
                  p0[1] + d_control * math.sin(direction0))
            p2 = (p3[0] - d_control * math.cos(direction3),
                  p3[1] - d_control * math.sin(direction3))
            
            segment = cubic_bezier(p0, p1, p2, p3, num_points=20)
            if i > 0:
                segment = segment[1:]
            path.extend(segment)
        return path
