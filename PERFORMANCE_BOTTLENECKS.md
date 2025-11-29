# 경로 생성 성능 병목 요약

| 우선순위 | 위치/기능 | 원인 | 영향 | 코드 위치 |
| --- | --- | --- | --- | --- |
| 1 | Bézier 제어점 밀어내기(`_push_away_from_obstacles`) | 충돌 구간을 찾기 위해 매 패스마다 `_find_intervals_bezier_hits` 재귀 분할 → 각 구간에서 P1/P2를 이동하며 최단 장애물 탐색 | 장애물·세그먼트 수에 비례해 반복 호출이 늘어나며 CPU 사용량 급증 | `bezier_utils.py` 180-240【F:src/path_follower_pkg/path_follower_pkg/bezier_utils.py†L180-L240】 |
| 2 | 코스트맵 제약 생성 | 코스트맵 셀을 stride로 샘플 후 모든 후보에 대해 경로-폴리라인 최소거리 계산, 경로에서도 stride만큼만 줄여 반복 → 셀·경로 길이에 따라 O(N×M) 스캔 | 로봇 주변에 셀/경로 점이 많을 때 constraint 빌드가 지연되고 베지어 재계산 주기가 느려짐 | `costmap_constraint_filter.py` 143-188【F:src/path_follower_pkg/path_follower_pkg/costmap_constraint_filter.py†L143-L188】 |
| 3 | RRT 브릿지 경로 | 구간마다 RRT를 실행하고, 세그먼트별 장애물 필터링·계산을 반복 | 클릭 시 장애물이 많거나 세그먼트가 많으면 RRT 호출이 누적돼 응답 지연 | `path_manager.py` 198-240【F:src/path_follower_pkg/path_follower_pkg/path_manager.py†L198-L240】 |

위 세 구간은 실제 경로 생성 지연을 가장 크게 만드는 연산 경로이며, 완화 없이는 반복 재계산 시 지터가 발생할 수 있습니다.
