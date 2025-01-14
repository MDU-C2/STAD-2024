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
gst-launch-1.0 udpsrc port=5600 caps="application/x-rtp" ! rtph264depay ! avdec_h264 ! videorate ! video/x-raw,framerate=1/1 ! videoconvert ! pngenc ! multifilesink location="frame.png"

python3 aruco.py
```
The simulation works with uav_cpp packages, to run uav_cpp please view the guide inside /uav_cpp

## Alternatively:

We already have a docker container which has everything set up and installed it is highly

recommended to use that container since the versions changes frequently.

In case of running it with docker. Docker does not provide a GUI with which you can see the gazebo

GUI. To solve this issue first install xserver for GUI.

Once you are inside container.

```
cd
```
```
cd PX4-Autopilot
make px4_sitl gazebo-classic
```

