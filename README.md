# ROS2 ORB SLAM3 V1.0 package

A ROS2 package for ORB SLAM3 V1.0. Focus is on native integration with ROS2 ecosystem. My goal is to provide a system that is capable of performing Navigation using Localization and Mapping features of ORB SLAM3.

## 0. Preamble
* This package builds [ORB-SLAM3](https://github.com/UZ-SLAMLab/ORB_SLAM3) V1.0 as a shared internal library. Comes included with a number of Thirdparty libraries [DBoW2, g2o, Sophus]
* g2o used is an older version and is incompatible with the latest release found here [g2o github page](https://github.com/RainerKuemmerle/g2o).
* This package differs from other ROS1 wrappers, thien94`s ROS 1 port and ROS 2 wrappers in GitHub by supprting/adopting the following
  * A separate python node to send data to the ORB-SLAM3 cpp node. This is purely a design choice.
  * At least C++17 and Cmake>=3.8
  * Eigen 3.3.0, OpenCV 4.2, latest release of Pangolin
* For newcomers in ROS2 ecosystem, this package serves as an example of building a shared cpp library and also a package with cpp nodes.
* May not build or work correctly in **resource constrainted hardwares** such as Raspberry Pi 4, Jetson Nano

## Testing platforms
1. AMD Ryzen 4500H, x86_64 bit architecture, Ubuntu 22.04 LTS (Jammy Jellyfish) and RO2 Humble Hawksbill (LTS)

## 1. Prerequisites
The following softwares must be installed before this package can be installed and used correctly

### Eigen3

```
sudo apt install libeigen3-dev
```

### Pangolin and configuring dynamic library path
We install Pangolin system wide and configure the dynamic library path so the necessary .so from Pangolin can be found by ros2 package during run time. More info here https://robotics.stackexchange.com/questions/105973/ros2-port-of-orb-slam3-can-copy-libdow2-so-and-libg2o-so-using-cmake-but-gettin

#### Install Pangolin

```
cd ~/Documents
git clone https://github.com/stevenlovegrove/Pangolin
cd Pangolin
./scripts/install_prerequisites.sh --dry-run recommended # Check what recommended softwares needs to be installed
./scripts/install_prerequisites.sh recommended # Install recommended dependencies
cmake -B build
cmake --build build -j4
sudo cmake --install build
```
#### Configure dynamic library

Check if ```/usr/lib/local``` is in the LIBRARY PATH
```
echo $LD_LIBRARY_PATH
```
If not, then perform the following 
```
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/lib/local
sudo ldconfig
```
Then open the ```.bashrc``` file in ```\home``` directory and add these lines at the very end
```
if [[ ":$LD_LIBRARY_PATH:" != *":/usr/local/lib:"* ]]; then
    export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH
fi
```
Finally, source ```.bashrc``` file 
```
source ~/.bashrc
```
 
### OpenCV
Ubuntu 22.04 by default comes with >OpenCV 4.2. Check to make sure you have at least 4.2 installed. Run the following in a terminal
```
python3 -c "import cv2; print(cv2.__version__)" 
```

## 2. Installation
1. In a new terminal move to home directory
```
cd ~
```
2. Create the ```vslam_ws``` workspace, and download this package as shown below.
```
mkdir -p ~/vslam_ws/src
cd ~/vslam_ws/src
git clone https://github.com/Mechazo11/ros2_orb_slam3.git
cd .. # make sure you are in ~/ros2_ws root directory
```
3. Source ROS2 Humble tools and run colcon build commands
```
source /opt/ros/humble/setup.bash
colcon build --symlink-install
```

4. Install Nav2 Package
```
sudo apt install ros-humble-navigation2
sudo apt install ros-humble-nav2-bringup
```

## 3. Perform Navigation using RGBD:

In one terminal [cpp node]
```
cd ~\vslam_ws
source install/setup.bash
ros2 run ros2_orb_slam3 rgbd ./src/ros2_orb_slam3/orb_slam3/Vocabulary/ORBvoc.txt.bin ./src/ros2_orb_slam3/orb_slam3/config/rgb-d/Gazebo_cam.yaml

```

In another terminal [Turtlebot3_Gazebo node]
```
cd ~\vslam_ws
source install/setup.bash
ros2 launch turtlebot3_gazebo navigation.launch.py
```

Now, Give a goal using Nav2_goal in RViz2 toolbar.

* The Nav2 is currently using a map that was previously created using ORB SLAM3. For creating a new map, follow Step.

In another terminal [Turtlebot3_Gazebo node]
```
cd ~\vslam_ws
source install/setup.bash
ros2 run turtlebot3_teleop ros2 run turtlebot3_teleop teleop_keyboard
```
Then use Keyboard to control the Robot to move around and record the Map, if the map is satisfactory cut the node using ctrl+c. This will save the map in both '.OSA' and '.TXT' files.

* '.OSA' file is for loading the map in ORB SLAM3 (See ros2_orb_slam3/orb_slam3/config/rgb-d/Gazebo_cam.yaml (System.LoadAtlasFromFile))
* '.TXT' file only contains the point cloud data in the Cartesian Space.

Use ros2_orb_slam3/src/txt_to_ply.py to convert the '.TXT' file into '.PLY'.

Then by using ros2_orb_slam3/src/ply_to_pgm_with_filter.py, convert the '.PLY' into '.PGM', which is used in Nav2 for publishing Map.

Now, provide the path to the new Map's Yaml file in the Navigation.launch.py.

## Ros2_orb_slam original package:

https://github.com/Mechazo11/ros2_orb_slam3.git

Thank you for taking the time in checking this project out. I hope it helps you out.