#!/bin/bash
source /opt/ros/jazzy/setup.bash
source ~/sentry_nav_26/install/setup.bash
echo "Starting planner under GDB..."
gdb -batch -ex run -ex bt -ex quit --args \
  ~/sentry_nav_26/install/sentry_planner/lib/sentry_planner/sentry_planner_node \
  --ros-args -r __node:=sentry_planner \
  --params-file ~/sentry_nav_26/install/sentry_planner/share/sentry_planner/config/planner_params.yaml
