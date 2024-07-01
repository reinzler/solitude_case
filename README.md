# Solitude Aerial test case

Steps to reproduce:
colcon build --packages-select px4_ros_com ros2_aruco px4_msgs && source install/local_setup.bash
~/src/PX4-Autopilot make px4_sitl gazebo-classic
MicroXRCEAgent udp4 -p 8888

To test the drone's flight in a circle
ros2 run px4_ros_com circle_flight.py

To test flight of a drone in a straight line and AAruco marker detection:
ros2 run px4_ros_com straight_flight.py
ros2 launch ros2_aruco aruco_recognition.launch.py

