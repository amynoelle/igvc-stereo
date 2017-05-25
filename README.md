# USMA IGVC Sterevision

### Usage Instructions
#### Set-up
Every time the Jetson board is turned off, the clock resets. This causes an issue with the ROS timestamps if you were to simply turn on the Jetson and Iggy and run the launch file. Use ntpdate to set the initial date and time then ntpd to keep them synchronized. In the first, ssh into the Jetson board with "ssh ubuntu@[Jetson_IP]" and the password is ubuntu (super secure).

#### NTP setup on the Jetson
To initalize the date time of the jetson tk-1 and restart ntp requests, modify the file /etc/init.d/rc.local by adding to the bottom:

( /etc/init.d/ntp stop
source /home/ubuntu/ntp_ip.cfg
echo "NTP_IP, $NTP_IP"
until ping -nq -c3 $NTP_IP; do
    echo "Waiting for network..."
    sleep 1
done
ntpdate -s time.nist.gov
/etc/init.d/ntp start )&

Create a config file in the home directory called /home/ubuntu/ntp_ip.cfg with one line in it:
NTP_IP=192.168.2.2

This IP address needs to be the IP address of the ROS_MASTER_URI
  
#### Tuning The Filters
Running the launch file that starts Iggy in a state where she is ready to run the course shoudl also start an rqt_reconfigure gui. This gui should have an option for zed_stereo, if not hit the refresh button and it should show up. The filters will not use the correct parameters until the rqt callback is invoked by changing one of the parameters, so go ahead and change one of the filter values by one just to see how well the current set of values performs.
    
#### Launch Files
There are two launch files in the launch foler. One is used to launch the code from the Jetson and one to launch the code from Iggy. You can copy the text of the igvc_stere.launch file into any other .launch file on Iggy and it will launch a zed-stereo node publishing the filtered point clouds. That file also contains a transform from zed_current_fram to map that places the point clouds where they belong relative to the camera's actual location on Iggy.

### Installation Instructions
#### Install ZED SDK
1. Get TK1 version of ZED SKD from:
  * https://www.stereolabs.com/developers/release/1.1.1a/
2. cd {Download locaiton}
3. chmod 777 ZED*
4. ./ZED

#### Install ROS
1. sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu trusty main" > /etc/apt/sources.list.d/ros-latest.list'
2. sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net --recv-key 0xB01FA116
3. sudo apt-get update
4. sudo apt-get install ros-indigo-ros-base
5. sudo apt-get install python-rosdep
6. sudo rosdep init
7. rosdep update
8. echo "source /opt/ros/indigo/setup.bash" >> ~/.bashrc
9. source ~/.bashrc
10. sudo apt-get install python-rosinstall

#### Install Rvz
1. sudo apt-get install ros-indigo-rviz
2. sudo apt-get install ros-indigo-robot-model
3. echo "unset GTK_IM_MODULE" >> ~/.bashrc
4. source ~/.bashrc

#### Install pcl
1. sudo add-apt-repository ppa:v-launchpad-jochen-sprickerhof-de/pcl
2. sudo apt-get update
3. sudo apt-get install libpcl-all

#### Install pcl-ros
1. sudo apt-get install ros-indigo-pcl-ros

#### Install zed-ros-wrapper
1. cd ~/[CATKIN_WOKRSPACE]/src
2. git clone https://github.com/stereolabs/zed-ros-wrapper 
3. cd ~/[CATKIN_WORKSPACE]
4. catkin_make zed-ros-wrapper
5. source ./devel/setup.bash

#### Recompile OpenCV
1. cd ~/[CATKIN_WORKSPACE]/src
2. git clone https://github.com/ros-perception/vision_opencv
3. cd ..
4. catkin_make --pkg vision_opencv

### ROS Networking
#### ON JETSON TK1:	
##### Jetson IP = 192.168.2.201
##### Remote IP = 192.168.2.200
1. echo export ROS_MASTER_URI=http://[REMOTE_PC_IP]:11311 >> ~/.bashrc
2. echo export ROS_HOSTNAME=[JETSON_IP] >> ~/.bashrc
3. echo export ROS_MASTER_URI=http://[REMOTE_PC_IP]:11311 >> ~/[CATKIN_WORKSPACE]/devel/setup.sh
4. echo export ROS_HOSTNAME=[JETSON_IP] >> ~/[CATKIN_WORKSPACE]/devel/setup.sh
5. export ROS_IP=[JETSON_IP]

#### ON REMOTE PC
1. echo export ROS_MASTER_URI=http://localhost:11311 >> ~/.bashrc
2. echo export ROS_HOSTNAME=[PC_IP] >> ~/.bashrc
3. echo export ROSLAUNCH_SSH_UNKNOWN = 1 >> ~/.bashrc
