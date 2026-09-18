# UI_ROS

Doosan M0609 설거지 로봇 HMI용 ROS2 보조 노드와 Flask UI를 한 저장소에 정리한 패키지입니다.

- ROS2 패키지 이름: `ui_ros`
- GitHub 폴더/저장소 이름: `UI_ROS`
- Launch 실행 대상: `home_open`, `ros_f_g`, `grip_error_stop_topic`
- Flask `app.py`: Launch에 포함하지 않고 별도 터미널에서 실행

## 폴더 구조

```text
UI_ROS/
├── launch/
│   └── ui_ros.launch.py
├── resource/
│   └── ui_ros
├── ui_ros/
│   ├── __init__.py
│   ├── home_gripper_control.py
│   ├── force_gripper_monitor.py
│   └── gripper_safety.py
├── ui/
│   ├── app.py
│   ├── requirements.txt
│   ├── templates/
│   │   └── index.html
│   └── static/
│       └── style.css
├── package.xml
├── setup.cfg
└── setup.py
```

## 1. 워크스페이스에 복사

```bash
cd ~/ws_cobot_pjt/ws_dsr/src
cp -r ~/Downloads/ui_ros .
```

기존에 같은 이름의 폴더가 있다면 먼저 백업하거나 삭제합니다.

## 2. Python 의존성

OnRobot 감시 노드와 UI에서 `pymodbus`를 사용합니다.

```bash
python3 -m pip install --user pymodbus flask
```

또는 UI 폴더의 requirements 사용:

```bash
cd ~/ws_cobot_pjt/ws_dsr/src/dish_washing_cobot/ui_ros/ui
python3 -m pip install --user -r requirements.txt
```

## 3. 빌드

Doosan 패키지가 있는 워크스페이스에서 실행합니다.

```bash
cd ~/ws_cobot_pjt/ws_dsr
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select ui_ros
source install/setup.bash
```

## 4. ROS2 노드 Launch 실행

먼저 Doosan bringup과 메인 식기세척 노드를 실행한 뒤 아래 명령을 실행합니다.

```bash
cd ~/ws_cobot_pjt/ws_dsr
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch ui_ros ui_ros.launch.py
```

Launch가 실행하는 노드:

| 실행 파일 | 기능 |
|---|---|
| `home_gripper_control` | 홈 이동 및 그리퍼 열기 토픽 수신 |
| `force_gripper_monitor` | 힘 센서와 그리퍼 상태 토픽 발행 |
| `gripper_safety` | OnRobot 안전 상태 감시, 정지 명령, 그리퍼 재시작 |

개별 실행도 가능합니다.

```bash
ros2 run ui_ros home_gripper_control
ros2 run ui_ros force_gripper_monitor
ros2 run ui_ros gripper_safety
```

## 5. Flask UI 별도 실행

Launch 터미널과 다른 터미널을 엽니다.

```bash
cd ~/ws_cobot_pjt/ws_dsr
source /opt/ros/humble/setup.bash
source install/setup.bash
cd src/dish_washing_cobot/ui_ros/ui
python3 app.py
```

브라우저 접속:

```text
http://127.0.0.1:5000
```

휴대폰 등 다른 기기에서는 터미널에 표시되는 PC의 LAN 주소를 사용합니다.

## 주요 토픽

| 토픽 | 타입 | 방향/기능 |
|---|---|---|
| `/dishwasher/robot/home` | `std_msgs/msg/Empty` | UI → 홈 이동 노드 |
| `/dishwasher/gripper/open` | `std_msgs/msg/Empty` | UI → 그리퍼 열기 노드 |
| `/dishwasher/gripper/restart` | `std_msgs/msg/Empty` | UI → OnRobot 전원 사이클 |
| `/dishwasher/gripper/safety_stop` | `std_msgs/msg/Bool` | 안전 감시 노드 → UI |
| `/dishwasher/force/all` | `std_msgs/msg/Float64MultiArray` | 힘 6축 값 |
| `/dishwasher/force/magnitude` | `std_msgs/msg/Float64` | 힘 크기 |
| `/dishwasher/gripper/state` | `std_msgs/msg/String` | DO2/DO3 기반 그리퍼 상태 |
| `/dishwasher/cmd` | `std_msgs/msg/String` | 작업 및 정지 명령 |

## 테스트 명령

```bash
ros2 topic pub --once /dishwasher/robot/home std_msgs/msg/Empty "{}"
ros2 topic pub --once /dishwasher/gripper/open std_msgs/msg/Empty "{}"
ros2 topic pub --once /dishwasher/gripper/restart std_msgs/msg/Empty "{}"
ros2 topic echo /dishwasher/gripper/safety_stop
ros2 topic echo /dishwasher/force/magnitude
ros2 topic echo /dishwasher/gripper/state
```

## 주의 사항

- `app.py`는 Launch에서 실행하지 않습니다.
- `home_open.py`와 `ros_f_g.py`는 `DSR_ROBOT2`, `DR_init`, `dsr_msgs2`가 설치된 Doosan 워크스페이스에서 실행해야 합니다.
- OnRobot Compute Box 기본 주소는 코드상 `192.168.1.1:502`, 장치 ID는 `65`입니다.
- 실제 장비에서 Launch를 실행하기 전에 Doosan bringup, 로봇 모드, 안전 상태를 확인하세요.
