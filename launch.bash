# stop the screen saver
killall light-locker

# launch the ros stuff
source /ros/install/setup.bash

xargs -P 3 -I {} sh -c {} <<'EOF'
ros2 launch bringup r1.launch.py
ros2 launch bringup r2.launch.py
ros2 launch bringup scenario_1.launch.py
EOF


