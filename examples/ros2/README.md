# Mighty ROS 2 Example

This example wraps the Python Mighty SDK as a small ROS 2 publisher.

It publishes:

- `/mighty/odom` (`nav_msgs/msg/Odometry`)
- `/mighty/imu` (`sensor_msgs/msg/Imu`)
- `/mighty/image_raw` (`sensor_msgs/msg/Image`, `mono8`)
- `/mighty/status` (`std_msgs/msg/String`)
- `/mighty/vio_state` (`std_msgs/msg/String`, JSON payload)
- `/tf` (`odom` -> `base_link`) when `publish_tf:=true`

The package is intentionally simple. It is meant as a starting point for ROS 2
users who want to confirm that Mighty Camera can feed a ROS graph before writing
a custom robot-specific adapter.

## Docker Build

From the repository root:

```bash
docker build -f examples/ros2/Dockerfile -t mighty-ros2 .
```

Run with host networking so the container can reach the USB Ethernet device at
`192.168.7.1`:

```bash
docker run --rm -it --network host mighty-ros2 \
  ros2 launch mighty_ros2 mighty_ros2.launch.py
```

Pass a custom host:

```bash
docker run --rm -it --network host mighty-ros2 \
  ros2 launch mighty_ros2 mighty_ros2.launch.py base_url:=http://192.168.7.1
```

## Native Build

Install ROS 2, then from a workspace:

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/asadm/mighty-protocol.git
cd ..
export PYTHONPATH="$PWD/src/mighty-protocol/python:$PYTHONPATH"
source /opt/ros/humble/setup.bash
colcon build --packages-select mighty_ros2 --base-paths src/mighty-protocol/examples/ros2/mighty_ros2
source install/setup.bash
ros2 launch mighty_ros2 mighty_ros2.launch.py
```

## Parameters

- `base_url`: optional explicit Mighty URL. Empty uses SDK defaults.
- `start_vio`: send `start_vio` when the node starts. Default `true`.
- `stop_vio_on_shutdown`: send `stop_vio` on shutdown. Default `false`.
- `publish_tf`: publish `odom` -> `base_link` transforms. Default `true`.
- `use_device_time`: use SDK device timestamps as ROS stamps. Default `false`.
- `frame_id`: odometry frame. Default `odom`.
- `child_frame_id`: body frame. Default `base_link`.
- `image_frame_id`: image frame. Default `camera`.

`use_device_time` defaults to false because Mighty timestamps are device
monotonic timestamps, not necessarily synchronized to ROS wall time. Leave it
false unless your system explicitly handles device time.

## Inspect Topics

```bash
ros2 topic list
ros2 topic echo /mighty/odom
ros2 topic echo /mighty/imu
ros2 topic echo /mighty/image_raw
ros2 topic echo /mighty/vio_state
```
