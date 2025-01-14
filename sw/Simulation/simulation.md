# Simulation Guide:

To install simulation(https://docs.px4.io/main/en/dev_setup/building_px4.html#gazebo-classic) follow the official guide.

```
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
make px4_sitl gazebo-classic
```
Using this command you should see the gazebo simulator coming up according to the official
documentation but it does not work sometimes. You have to install gazebo classic using below
command.

```
curl -sSL http://get.gazebosim.org | sh
```
```
cd
```
```
cd PX4-Autopilot
make px4_sitl gazebo-classic
```
## To get the images from simulation run the following commands inside /python-files
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
