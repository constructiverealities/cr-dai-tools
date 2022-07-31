#!/bin/bash
set -e

# setup ros environment
source "/opt/ros/$ROS_LOAD_DISTRO/setup.bash"
exec "$@"
