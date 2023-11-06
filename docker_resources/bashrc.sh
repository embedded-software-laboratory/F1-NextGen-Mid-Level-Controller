# Source ROS
source /home/pi/ros2_ws/install/setup.bash

# Cross-compilation command for colcon
cross-colcon-build() {
  colcon build \
    "$@" \
    --merge-install \
    --cmake-force-configure \
    --cmake-args \
    -DCMAKE_TOOLCHAIN_FILE=/opt/raspi-toolchain.cmake \
    -DCMAKE_VERBOSE_MAKEFILE:BOOL=ON \
    -DTHIRDPARTY=ON \
    -DBUILD_TESTING:BOOL=OFF
}

scp-ros2_ws() {
  if [ $# -ne 2 ]; then
    echo "Usage: scp-ros2_ws <NAME> <IP_ADDRESS>"
  else
    local name="$1"
    local address="$2"
    scp -r /home/pi/ros2_ws/install $name@$address:~/ros2_ws/
  fi
}

scp-vehicle_ws() {
  if [ $# -ne 2 ]; then
    echo "Usage: scp-vehicle_ws <NAME> <IP_ADDRESS>"
  else
    local name="$1"
    local address="$2"
    scp -r /home/pi/vehicle_ws/install $name@$address:~/vehicle_ws/
  fi
}
