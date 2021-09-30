FROM kristoferb/spweb_ros2:galactic

RUN apt-get update -qqy \
    && apt-get install -qqy ros-galactic-xacro

COPY . /ros/
RUN . /opt/ros/$ROS_DISTRO/setup.sh && cd /ros/ && colcon build
