#!/usr/bin/env bash
set -e

source "/opt/ros/${ROS_DISTRO:-jazzy}/setup.bash"

if [ -f "/workspace/sentry_nav_26/install/setup.bash" ]; then
    source "/workspace/sentry_nav_26/install/setup.bash"
fi

exec "$@"
