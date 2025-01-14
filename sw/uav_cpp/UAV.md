# Running UAV Code:
Build all the packages.
It is important to build all the packages if you want to use computer vision.
```
colcon build

```
## Run offboard control
To get offboard control run the code

```
source install/setup.bash
ros2 run uav_cpp uav_main udp://:14540

```
If you are running it on hardware then run the following command to run aruco tracker on UGV
```
ros2 run aruco_tracker aruco_tracker
```
If you are running with simulation run the following commands inside /python-files
```
gst-launch-1.0 udpsrc port=5600 caps="application/x-rtp" ! rtph264depay ! avdec_h264 ! videorate ! video/x-raw,framerate=1/1 ! videoconvert ! pngenc ! multifilesink location="frame.png"

python3 aruco.py
```
make sure that before everything you have px4 running with gazebo-classic
