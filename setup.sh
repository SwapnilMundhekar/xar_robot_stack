#!/usr/bin/env bash
#  ────────────────────────────────────────────────────────────
#  setup.sh  –  bootstrap a fresh ROSbot 3 Pro (Ubuntu 24.04)
#  Installs minimal ROS 2 Humble, Python deps, builds workspace,
#  and adds the overlay to ~/.bashrc · 18 Jun 2025
#  ────────────────────────────────────────────────────────────
set -e
WS="$PWD/ros2_ws"

echo -e "\n📦  STEP 1  –  ensure ROS 2 Humble base is present"
if ! dpkg -s ros-humble-ros-base >/dev/null 2>&1; then
  sudo apt update
  sudo apt install -y curl gnupg lsb-release
  sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
       -o /usr/share/keyrings/ros-archive-keyring.gpg
  echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
  http://packages.ros.org/ros2/ubuntu jammy main" | \
  sudo tee /etc/apt/sources.list.d/ros2.list
  sudo apt update
  sudo apt install -y ros-humble-ros-base python3-colcon-common-extensions
fi

echo -e "\n🐍  STEP 2  –  install Python libs for rosbridge"
sudo apt install -y python3-twisted python3-tornado python3-autobahn

echo -e "\n🔧  STEP 3  –  build workspace ($WS)"
source /opt/ros/humble/setup.bash
cd "$WS"
colcon build --symlink-install

echo -e "\n📚  STEP 4  –  auto-source overlay in future shells"
SRC_LINE="source $WS/install/setup.bash"
grep -qxF "$SRC_LINE" ~/.bashrc || echo "$SRC_LINE" >> ~/.bashrc

echo -e "\n✅  All done!  Open a new terminal and run:"
echo "   ros2 launch xar_nav_manager nav_manager.launch.py slam:=true"
