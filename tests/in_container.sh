#!/usr/bin/env bash
# Runs inside ros:jazzy-ros-base (tests/docker_check.sh): what the robot computer does, minus the CAN.
#   /src/sim2real  this repository (read-only)      /ik  team-jax/IK (read-only, optional)
set -e
apt-get update -qq && apt-get install -y -qq git python3-can python3-pip >/dev/null
mkdir -p /ws/src && cd /ws/src
git clone -q --depth 1 https://github.com/team-jax/ak45_can.git
git clone -q --depth 1 https://github.com/team-jax/ak45_ros2.git
cp -r /src/sim2real /ws/src/SimToReal && rm -rf /ws/src/SimToReal/.git
source /opt/ros/jazzy/setup.bash && cd /ws
colcon build --symlink-install 2>&1 | grep -E -i "warning|error|Finished|Failed" || true
source install/setup.bash
ros2 pkg executables sim2real
# Not piped into head: an early close is a BrokenPipeError traceback.
ros2 launch sim2real sim2real.launch.py --show-args > /tmp/args.txt && grep -c default /tmp/args.txt
echo "{\"joint_names\": [\"elbow\"], \"dt\": 0.02, \"positions\": [[0.0], [0.01]], \"status\": \"playable\", \"delivered_in_sim\": true}" > /tmp/t.json
ros2 run sim2real play_trajectory /tmp/t.json --dry-run || echo "play_trajectory dry run exit $?"
cd /ws/src/SimToReal && python3 tests/check_protocol.py --ik /nonexistent | tail -1
python3 tests/check_node.py | tail -1

if [ -d /ik/motion1 ]; then
  echo "== pick_place with IK planning (MuJoCo on Ubuntu 24.04, Python $(python3 -V | cut -d' ' -f2))"
  # --user: next to, not over, the Debian packages (pip cannot uninstall typing_extensions 4.10 from apt)
  pip install -q --user --break-system-packages mujoco==3.10.0 gymnasium scipy 2>&1 | grep -v "WARNING" | tail -1 || true
  cd /tmp
  ros2 run sim2real pick_place --ik /ik --cube 0.40 -0.30 0 --target 0.05 -0.27 --dry-run > /tmp/pp_far.log 2>&1 || echo "unreachable exit $?"
  tail -1 /tmp/pp_far.log
  ros2 run sim2real sim2real_node --ros-args -p servo_interface:=virtual -p servo_channel:=t \
      -p max_speed_deg_s:=300.0 > /tmp/node.log 2>&1 &
  python3 /ws/src/SimToReal/tests/fake_ak45.py > /tmp/fake.log 2>&1 & FAKE=$!
  sleep 2
  ros2 run sim2real pick_place --ik /ik --cube 0.10 -0.30 0 --target 0.05 -0.27 --force \
      --max-speed 300 --hold 1 > /tmp/pp.log 2>&1 && echo "pick_place played: exit 0" || echo "pick_place exit $?"
  tail -2 /tmp/pp.log
  kill $FAKE; sleep 1
  python3 /ws/src/SimToReal/tests/fake_ak45.py --freeze ak45_4 > /tmp/fake2.log 2>&1 & FAKE=$!
  sleep 2
  ros2 run sim2real pick_place --ik /ik --cube 0.10 -0.30 0 --target 0.05 -0.27 --force \
      --max-speed 300 --hold 1 > /tmp/pp2.log 2>&1 && echo "frozen motor: exit 0 (WRONG)" || echo "frozen motor: exit $?"
  tail -2 /tmp/pp2.log
fi
