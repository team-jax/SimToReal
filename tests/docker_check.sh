#!/usr/bin/env bash
# Build ak45_can + ak45_ros2 + this package in the ros:jazzy image (Ubuntu 24.04) and run the checks,
# then pick_place end to end with IK planning if team-jax/IK sits next to this repository (or $IK).
# No CAN hardware: Docker Desktop for Mac has no AF_CAN, so ak45_node itself cannot start here;
# tests/fake_ak45.py plays its part and the servos go on a python-can virtual bus.
#   tests/docker_check.sh
set -euo pipefail
here="$(cd "$(dirname "$0")/.." && pwd)"
ik="${IK:-$here/../IK}"
mounts=(-v "$here":/src/sim2real:ro)
[ -d "$ik/motion1" ] && mounts+=(-v "$(cd "$ik" && pwd)":/ik:ro)
docker run --rm "${mounts[@]}" ros:jazzy-ros-base bash /src/sim2real/tests/in_container.sh
