# 경로 생성 지연 원인 분석 (최신 코드 기준)

`local_bezier`와 글로벌 Bézier 경로 생성이 느려지는 경로를 **비용이 큰 순서**로 정리했다. 각 항목은 실제 코드 위치와 왜 계산이 커지는지, 언제 호출되는지까지 포함한다.

| 우선순위 | 위치/라인 | 원인 | 설명 |
| --- | --- | --- | --- |
| 1 | `bezier_utils._push_away_from_obstacles` / `_find_intervals_bezier_hits` (L127-L224, L179-L224) | Bézier-장애물 충돌 분할 + 반복 패스 | 세그먼트마다 convex-hull 브로드페이즈 후에도 겹침이 있으면 최대 14패스 동안 충돌 구간을 재귀 분할하고 구간마다 3지점 평가를 한다. 장애물·세그먼트가 많을 때 연산이 폭증한다. |
| 2 | `bezier_utils.generate_bezier_from_waypoints` (L530-L633) & `split_global_to_local_bezier` (L446-L527) | 전 구간 Bézier 체인 + 15Hz 로컬 재계산 | 글로벌은 모든 세그먼트에 대해 1번 루프를 다시 돌리고, 로컬은 15 Hz로 lookahead 구간을 다시 자른 뒤 길이 기반 샘플링(최소 10점)과 충돌 보정을 반복해 누적 비용이 크다. |
| 3 | `costmap_constraint_filter.build_constraints` / `build_constraints_for_path` (L143-L188, L190-L245) | 코스트맵 점 vs 경로 전수 거리 계산 | stride 이후 남은 모든 점에 대해 polyline 최근접 거리(O(N×M))와 최근접 인덱스를 구한다. 코스트맵·경로가 길수록 반복 비용이 급증하며, 로봇 주변/전역 두 번 실행된다. |
| 4 | `path_manager._update_path` (L129-L199) | 8 cm 고정 샘플링 + 즉시 재보간 | 클릭·모드 변경·코스트맵 replan마다 전체 경로를 ds=0.08 간격으로 다시 생성하고 방향을 계산한다. Bézier/스플라인 모두 점 수가 많아지고 1~3번 병목을 다시 호출한다. |
| 5 | `path_manager._rrt_bridge_waypoints` (L209-L240) + `rrt_planner.plan` (L36-L143) | 세그먼트별 최대 900회 RRT 확장 | `local_bezier` + 장애물 있을 때 각 세그먼트를 RRT로 잇는데, 매번 최대 900 샘플·충돌검사를 돌린다. 세그먼트가 여러 개면 병목이 중첩된다. |
| 6 | `follower_node._maybe_refresh_costmap_constraints` (L256-L301) | 코스트맵 수신 시 반복 replan | costmap 토픽을 받을 때마다 제약/장애물을 재계산해 PathManager replan을 트리거한다. replan interval(기본 0.8 s) 내에는 건너뛰지만, 토픽 주기가 빠르면 3~5 항목이 자주 호출된다. |

## 요약
- 핵심 병목은 **Bézier 충돌 보정(1,2)**과 **코스트맵-경로 전수 거리 계산(3)**이며, 두 모듈이 반복 호출될 때 지연이 가장 크다.
- **고정 샘플링 + 잦은 replan(4,6)**이 상위 병목을 빈번히 다시 실행하게 만들어 체감 성능이 떨어진다.
- `local_bezier`에서 장애물이 많을 경우 **세그먼트별 RRT(5)**까지 겹치면 클릭 후 경로 생성이 눈에 띄게 느려진다.
