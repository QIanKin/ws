#!/usr/bin/env bash
# Quick wrapper for teleop_twist_keyboard remapped to /pmocha/cmd_vel.
# Usage:  ros2 run pmocha_experiments teleop.sh
#         (or override namespace)  NAMESPACE=robot1 ros2 run pmocha_experiments teleop.sh

set -e
NAMESPACE="${NAMESPACE:-pmocha}"
echo "[pmocha teleop] publishing to /${NAMESPACE}/cmd_vel"
exec ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args -r /cmd_vel:=/${NAMESPACE}/cmd_vel
