# 🧭 tm_centerline_nav – ROS2 Centerline Waypoint Navigator

**ROS 2 + Nav2 + tf2** 를 이용해,  
지도 위에 미리 정의된 포인트들(1, 2, 3, …)로 로봇을 보내는 **센터라인 기반 내비게이션 노드**입니다. :contentReference[oaicite:0]{index=0}  

- `/pinky_tmproject/target` 토픽에 **번호(Int32)** 를 보내면
- YAML에 정의된 좌표로 향하는 **경유점 경로(centerline 경로)** 를 만들고
- `nav2_simple_commander.BasicNavigator` 의 `followWaypoints()` 로 주행합니다.
- RViz에는 각 포인트의 번호와 위치를 마커로 표시합니다.

---

## 🇰🇷 Korean Version

### 📌 개요

이 노드는 **Nav2가 이미 활성화된 환경**에서,  
정의된 포인트들(예: 1~5번)을 **센터라인(가상의 세로축)** 을 따라 이동하도록 만드는 내비게이션 보조 패키지입니다.

- **패키지(파이썬 기준)**: `pinky_tmproject` 안의 `tm_centerline_nav` 노드
- **내비게이션 제어**: `nav2_simple_commander.BasicNavigator`
- **현재 위치 조회**: `tf2_ros` (`map` → `base_link` TF)
- **경로 형태**: 항상 `x = center_x` 세로축을 **한 번 경유**하는 “ㄱ자/┐/┘” 형태의 경로

---

### 🧠 동작 개념 (센터라인 주행)

YAML에 정의된 목표 포인트를 `goal = (goal_x, goal_y)` 라고 할 때,  
센터라인 `x = center_x` 를 경유하는 경로를 **3개 waypoint**로 만듭니다. :contentReference[oaicite:1]{index=1}  

1. **현재 위치 → centerline 수평 이동**

   - 시작 포즈: TF(`map`←`base_link`)에서 조회  
   - 첫 번째 waypoint:  
     `A = (center_x, start_y)`  
   - yaw는 `start → A` 방향으로 설정

2. **centerline 따라 상/하 이동**

   - 두 번째 waypoint:  
     `B = (center_x, goal_y)`  
   - goal_y가 위쪽이면 `+π/2`, 아래쪽이면 `-π/2` 로 yaw 설정

3. **목표 x로 수평 이동**

   - 세 번째 waypoint:  
     `C = (goal_x, goal_y)`  
   - center_x보다 오른쪽이면 yaw = 0, 왼쪽이면 yaw = π

이렇게 만든 `[A, B, C]` 리스트를 `BasicNavigator.followWaypoints()`에 넘겨  
**항상 일정한 세로축을 지나가는 경유 경로**를 형성합니다.

---

### 🔌 토픽 인터페이스

노드 클래스: `TmCenterlineNav(Node)` :contentReference[oaicite:2]{index=2}  

**구독(Subscribe)**

- `/pinky_tmproject/target` (`std_msgs/Int32`)
  - 값: 1, 2, 3 …  
  - 해당 번호에 대응하는 포인트를 YAML에서 찾아 경로 생성 후 주행 시작
  - 이미 주행 중이면 기존 task를 `cancelTask()` 한 뒤 새 목표로 업데이트

- `/pinky_tmproject/stop` (`std_msgs/Bool`)
  - `True` 수신 시:
    - Nav2 task 취소(`cancelTask()`)
    - 내부 상태 `navigating = False` 로 변경
    - 상태 메시지 발행

**발행(Publish)**

- `/pinky_tmproject/status` (`std_msgs/String`)
  - 한글 상태 로그:
    - `"초기화 완료. ..."`
    - `"N번 포인트로 가는 중입니다."`
    - `"멈췄습니다."`
    - `"목표에 도달하지 못했습니다(실패)."` 등

- `/pinky_tmproject/points_markers` (`visualization_msgs/MarkerArray`)
  - RViz용 포인트 마커:
    - 각 포인트 번호 텍스트 (TEXT_VIEW_FACING, 빨간색)
    - 위치 점(SPHERE, 파란색 작은 구)
  - QoS: `TRANSIENT_LOCAL` 이라 RViz를 나중에 띄워도 최근 마커를 한 번에 받아볼 수 있음

---

### ⚙️ 파라미터 & YAML 포맷

노드 파라미터: :contentReference[oaicite:3]{index=3}  

- `points_yaml` (string)
  - 포인트 정의 YAML 경로
  - 비어 있으면: `get_package_share_directory('pinky_tmproject')/config/points.yaml` 사용
- `frame_id` (string, default: `"map"`)
  - 목표 포즈 및 마커 좌표의 기준 프레임
- `base_frame` (string, default: `"base_link"`)
  - 로봇 본체 TF 프레임 이름

YAML 예시 (points.yaml):

```yaml
frame_id: "map"
base_frame: "base_link"

center_x: 0.0
goal_tolerance_xy: 0.25
goal_tolerance_yaw: 0.5
marker_scale: 0.25

points:
  "1": { x: 0.5, y:  0.0 }
  "2": { x: 1.0, y:  0.5 }
  "3": { x: 1.0, y: -0.5 }
  "4": { x: 1.5, y:  0.5 }
  "5": { x: 1.5, y: -0.5 }
코드에서는 "1", "2" 같은 문자열 키를 정수형으로 변환해서 사용합니다. ({int(k): v for k, v in raw_points.items()})

📦 의존성 / 사전 준비
ROS 2 (예: Humble / Jazzy)

Nav2 스택 및 맵 서버 실행

nav2_simple_commander 파이썬 패키지

TF 트리:

frame_id (예: map)

base_frame (예: base_link)
사이의 변환이 계속 브로드캐스트되고 있어야 함

예: Nav2 활성화 후에 이 노드를 실행해야 정상 동작합니다.

▶ 실행 예시 (예시 코드 / 실제 launch 에 맞게 수정)
1) Nav2 + 맵 bringup (예시)

bash
Copy code
# 예: 자체 nav2 bringup launch
ros2 launch pinky_bringup nav2_view.launch.xml map:=/path/to/map.yaml
2) tm_centerline_nav 노드 실행 (예시)

bash
Copy code
# 실제 executable/launch 이름에 맞게 수정해서 사용
ros2 run pinky_tmproject tm_centerline_nav
# 또는
ros2 launch pinky_tmproject tm_centerline_nav.launch.py
3) 목표 포인트 보내기

bash
Copy code
# 1번 포인트로 이동
ros2 topic pub /pinky_tmproject/target std_msgs/Int32 "data: 1"

# 주행 중단
ros2 topic pub /pinky_tmproject/stop std_msgs/Bool "data: true"
🧪 RViz 표시
Fixed Frame: map

MarkerArray 디스플레이 추가 후 /pinky_tmproject/points_markers 선택

각 포인트 번호와 작은 구가 맵 위에 표시됩니다.

🌐 English Version
🧾 Overview
tm_centerline_nav is a ROS 2 waypoint navigation helper node built on top of Nav2 + tf2 + nav2_simple_commander. 
tm_navigator


It:

Loads goal points from a YAML file

Listens to an integer target topic (e.g., 1–5)

Builds a centerline-based path (always passing through x = center_x)

Sends the resulting waypoints to Nav2’s BasicNavigator (followWaypoints)

Publishes RViz markers for all defined points

🧠 Behavior (Centerline Path)
Given a target point (goal_x, goal_y), the node builds a 3-step path:

From current pose to centerline

Current pose from TF (frame_id ← base_frame)

First waypoint A = (center_x, start_y)

Heading is set from start to A

Move along the centerline vertically

Second waypoint B = (center_x, goal_y)

Heading is +π/2 (upwards) or −π/2 (downwards)

Move horizontally to final X

Third waypoint C = (goal_x, goal_y)

Heading is 0 (to the right) or π (to the left)

This yields a “L-shaped” or “┐/┘-style” path that always passes the centerline (x = center_x) before reaching the final goal.

🔌 Topics
Subscribed

/pinky_tmproject/target (std_msgs/Int32)

Value N selects the N-th point from the YAML file.

If already navigating, the current task is canceled and replaced.

/pinky_tmproject/stop (std_msgs/Bool)

True cancels the current Nav2 task and stops navigation.

Published

/pinky_tmproject/status (std_msgs/String)

Human-readable status messages (in Korean).

/pinky_tmproject/points_markers (visualization_msgs/MarkerArray)

Text labels (IDs) and small spheres for each point.

QoS: TRANSIENT_LOCAL, so RViz can receive markers even if opened later.

⚙ Parameters & YAML
Node parameters:

points_yaml (string): path to the points YAML

If empty, falls back to share/pinky_tmproject/config/points.yaml

frame_id (default "map"): global frame for goals/markers

base_frame (default "base_link"): robot base frame

YAML structure:

yaml
Copy code
frame_id: "map"
base_frame: "base_link"

center_x: 0.0
goal_tolerance_xy: 0.25
goal_tolerance_yaw: 0.5
marker_scale: 0.25

points:
  "1": { x: 0.5, y:  0.0 }
  "2": { x: 1.0, y:  0.5 }
  "3": { x: 1.0, y: -0.5 }
The node converts "1", "2" … to integers for internal lookup.

📦 Dependencies / Requirements
ROS 2 (Humble / Jazzy etc.)

Nav2 stack running and active

nav2_simple_commander Python module

Valid TF between frame_id (map) and base_frame (base_link)

The node expects Nav2 to be already active when it starts.

▶ Example Usage
Note: adjust executable and launch names to your actual package configuration.

1) Bring up Nav2 with a map

bash
Copy code
ros2 launch pinky_bringup nav2_view.launch.xml map:=/path/to/map.yaml
2) Run the centerline navigator

bash
Copy code
ros2 run pinky_tmproject tm_centerline_nav
# or
ros2 launch pinky_tmproject tm_centerline_nav.launch.py
3) Send goals and stop commands

bash
Copy code
ros2 topic pub /pinky_tmproject/target std_msgs/Int32 "data: 1"
ros2 topic pub /pinky_tmproject/stop   std_msgs/Bool  "data: true"
🧪 RViz Integration
Set Fixed Frame to map

Add a MarkerArray display for /pinky_tmproject/points_markers

You’ll see numbered labels and spheres at each defined point.

🔧 Technical Summary
Node name: tm_centerline_nav

Core libraries:

rclpy, tf2_ros, nav2_simple_commander.BasicNavigator

geometry_msgs/PoseStamped, visualization_msgs/MarkerArray

Main logic:

Subscribe to target index → build centerline waypoints → followWaypoints

Query TF for current pose

Visualize points in RViz

📜 License
This node is primarily intended for personal learning and robotics experiments.
For any commercial deployment, please review Nav2 / ROS 2 / dependency licenses and your own project’s policy.

