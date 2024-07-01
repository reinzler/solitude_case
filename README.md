# Solitude Aerial test case

Steps to reproduce:
<br>```colcon build --packages-select px4_ros_com ros2_aruco px4_msgs && source install/local_setup.bash```<br/>
<br>From **/src/PX4-Autopilot**</br>
```make px4_sitl gazebo-classic```<br/>
```MicroXRCEAgent udp4 -p 8888```<br/>
<br>To test the drone's flight in a circle<br/>
```ros2 run px4_ros_com circle_flight.py``` <br/>

To test flight of a drone in a straight line and Aruco marker detection:
<br>```ros2 run px4_ros_com straight_flight.py```<br/>
```ros2 launch ros2_aruco aruco_recognition.launch.py```<br/>

