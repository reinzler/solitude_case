# solitude_test
Solitude Aerial test case


git clone https://github.com/PX4/PX4-Autopilot
git clone https://github.com/PX4/px4_msgs
git clone --recursive https://github.com/Auterion/px4-ros2-interface-lib

pip3 install kconfiglib
pip3 install symforce

ros2 run example_mode_manual_cpp example_mode_manual
make px4_sitl gz_x500
~/addons/src/PX4-Autopilot make px4_sitl gazebo-classic

micro-xrce-dds-agent udp4 -p 8888 / MicroXRCEAgent udp4 -p 8888
ros2 run px4_ros_com offboard_control.py
ros2 run px4_ros_com circle_flight.py

./generate_markers_model.py -i /home/vadim/Downloads/gazebo_models/marker.png -s 2000 -w 1000
PX4_SIM_WORLD=iris_irlock.world make px4_sitl gazebo-classic



export PX4_HOME_LAT=59.92545296605414
export PX4_HOME_LON=30.358049691155855
export PX4_HOME_ALT=28.5


ros2 run rqt_image_view rqt_image_view
ros2 launch ros2_aruco aruco_recognition.launch.py
colcon build --packages-select px4_ros_com ros2_aruco 
source /home/vadim/solitude_test/install/local_setup.bash

QGroundControl
place QGroundControl AppImage to /usr/local/bin
QGroundControl.AppImage