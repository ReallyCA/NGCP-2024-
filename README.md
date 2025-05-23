**PREREQUISITES**

This guide assumes you have already installed both git and QGroundControl

**FIRST STEPS**

Installing Ros2

``
sudo apt install software-properties-common
``

``
sudo add-apt-repository universe
``

``
sudo apt update && sudo apt install curl -y
``

``
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
``

``
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
``

``
sudo apt update && sudo apt upgrade
``

``
sudo apt install ros-humble-desktop
``

``
sudo apt install ros-dev-tools
``


**RUNNING THE PROGRAM**

Run QGroundControl

**PX4 Gazebo Fixed Wing [OPTIONAL, YOU CAN RUN IT HEADLESS]**

``
cd PX4-Autopilot
``

``
make px4_sitl gz_rc_cessna
``


**MicroXRDEagent**

``
MicroXRCEAgent udp4 -p 8888
``


**Ros 2 Telemetry**

``
source install/local_setup.bash
``

``
ros2 launch px4_ros_com sensor_combined_listener.launch.py
``
