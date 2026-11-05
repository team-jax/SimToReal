#!/usr/bin/env bash
# Build ak45_can + ak45_ros2 + this package in the ros:jazzy image (Ubuntu 24.04) and run
# tests/check_node.py. No CAN hardware: Docker Desktop for Mac has no AF_CAN, so ak45_node itself
# cannot start here; the test plays its part.
#   tests/docker_check.sh
set -euo pipefail
here="$(cd "$(dirname "$0")/.." && pwd)"
docker run --rm -v "$here":/src/sim2real:ro ros:jazzy-ros-base bash -c '
set -e
apt-get update -qq && apt-get install -y -qq git python3-can >/dev/null
mkdir -p /ws/src && cd /ws/src
git clone -q --depth 1 https://github.com/team-jax/ak45_can.git
git clone -q --depth 1 https://github.com/team-jax/ak45_ros2.git
cp -r /src/sim2real /ws/src/SimToReal && rm -rf /ws/src/SimToReal/.git
source /opt/ros/jazzy/setup.bash && cd /ws
colcon build --symlink-install 2>&1 | grep -E -i "warning|error|Finished|Failed" || true
source install/setup.bash
ros2 pkg executables sim2real
echo "{\"joint_names\": [\"elbow\"], \"dt\": 0.02, \"positions\": [[0.0], [0.01]]}" > /tmp/t.json && ros2 run sim2real play_trajectory /tmp/t.json --dry-run
ros2 launch sim2real sim2real.launch.py --show-args > /tmp/args.txt && grep -c default /tmp/args.txt  # no pipe into head: an early close is a BrokenPipeError traceback
cd /ws/src/SimToReal && python3 tests/check_protocol.py --ik /nonexistent | tail -1
python3 tests/check_node.py
'
