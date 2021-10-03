FROM kristoferb/spweb_ros2:galactic

RUN apt-get update -qqy \
    && apt-get install -qqy \
       ros-galactic-xacro \
       nano \
       firefox

COPY . /ros/
RUN . /opt/ros/$ROS_DISTRO/setup.sh && cd /ros/ && colcon build

COPY launch.bash launch_simulation.bash launch_simulation_no_rviz.bash /
RUN chmod +x launch.bash launch_simulation.bash launch_simulation_no_rviz.bash &&\
    echo "source /opt/ros/galactic/setup.bash" >> ~/.bashrc &&\
    echo "source /ros/install/setup.bash" >> ~/.bashrc