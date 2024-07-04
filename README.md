# Solitude Aerial test case

Steps to reproduce.
<br>Firstly, install all required packages and software.<br/>
* **PX4 - ROS2 interface lib**
<br>```cd $ros_workspace/src```<br/>
```git clone --recursive https://github.com/Auterion/px4-ros2-interface-lib```<br/>
```cd ..```<br/>
```colcon build```<br/>
```source install/setup.bash```<br/>
<br><br/>
* **Micro-XRCE-DDS-Agent**
<br>```git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git```<br/>
```cd Micro-XRCE-DDS-Agent```<br/>
```mkdir build && cd build```<br/>
_On Linux, inside of the build folder, execute the following commands:_
<br>```cmake ..```<br/>
```make```<br/>
```sudo make install```<br/>
<br><br/>
* **QGroundControl**
<br>```sudo usermod -a -G dialout $USER```<br/>
```sudo apt-get remove modemmanager -y```<br/>
```sudo apt install gstreamer1.0-plugins-bad gstreamer1.0-libav gstreamer1.0-gl -y```<br/>
```sudo apt install libfuse2 -y```<br/>
```sudo apt install libxcb-xinerama0 libxkbcommon-x11-0 libxcb-cursor0 -y```<br/>
_Download and install QGroundControl_
<br>```https://d176tv9ibo4jno.cloudfront.net/latest/QGroundControl.AppImage ```<br/>
```chmod +x ./QGroundControl.AppImage```<br/>
_To be able to run QGroundControl from terminal, place QGroundControl app image to_ **/usr/local/bin/**
<br><br/>
* **PX4**
<br>```git clone https://github.com/PX4/PX4-Autopilot.git --recursive```<br/>
```bash ./PX4-Autopilot/Tools/setup/ubuntu.sh```<br/>
```cd PX4-Autopilot/```<br/>
```make px4_sitl```<br/>
```pip3 install --user -U empy==3.3.4 pyros-genmsg setuptools```<br/>
<br><br/>
* **PX4 messages and PX4_ros_com**
<br>```cd src/```</br>
```git clone https://github.com/PX4/px4_msgs.git```</br>
```git clone https://github.com/PX4/px4_ros_com.git```</br>
```colcon build --packages-select px4_ros_com px4_msgs && source install/local_setup.bash```<br/>
<br><br/>
* **Gazebo classic**
<br>```sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list' ```</br>
```wget https://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -```</br>
```sudo apt-get update```</br>
```sudo apt-get install gazebo11```<br/>
_If fails:_
<br>```sudo apt-get install gazebo```<br/>
_Gazebo ROS Libraries_
<br>```sudo apt install ros-<ROS_DISTRO>-gazebo-ros-pkgs```</br>
<br><br/>
* **ROS2 aruco**
_<br>Clone from current repo and install python libs:_
<br>```pip3 install opencv-contrib-python==4.9.0.80 transforms3d```</br>
_Build from your **src/** directory_:
<br>```colcon build --packages-select ros2_aruco ros2_aruco_interfaces && source install/local_setup.bash```</br>


<br>After installation, test:<br/>
From **/src/PX4-Autopilot**</br>
```make px4_sitl gazebo-classic```<br/>
<br>From different shell</br>
```MicroXRCEAgent udp4 -p 8888```<br/>
<br>From different shell</br>
```QGroundControl.AppImage```<br/>
<br>From different shell
<br>To test the drone's flight in a circle<br/>
```ros2 run px4_ros_com circle_flight.py``` <br/>
<br>From different shell</br>
To test flight of a drone in a straight line and Aruco marker detection:
<br>```ros2 run px4_ros_com straight_flight.py```<br/>
<br>From different shell</br>
```ros2 launch ros2_aruco aruco_recognition.launch.py```<br/>

