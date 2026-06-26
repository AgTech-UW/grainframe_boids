#!/usr/bin/env bash
set -e

# Make every new terminal automatically know about ROS 2
echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc

# And automatically find our package once it's built
echo 'for f in /workspaces/*/install/setup.bash; do [ -f "$f" ] && source "$f"; done' >> /root/.bashrc

echo "bootstrap done"