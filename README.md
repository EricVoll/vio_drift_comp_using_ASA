# Visual Inertial Odometry (VIO) drift compensation using Azure Spatial Anchors

This repo contains the ROS side code of my Semester Project at ETH's Autonomous Systems Lab (ASL), were I implemeneted VIO drift compensation using Azure Spatial Anchors for omnidirectional drones. The report with in-depth descriptions can be found [here](https://github.com/EricVoll/vio_drift_comp_using_ASA/blob/master/ThesisReport.pdf).

The Mixed Reality side of this project is located [here](https://github.com/EricVoll/ethz_asl_semester_project) and you can find a video of it [here](https://youtu.be/SxmkRreG5j8).

# Setup
1. Install ROS melodic
2. Setup catkin workspace using:
 ```
 cd ~ && mkdir -p catkin_ws/src && cd catkin_ws
 catkin config --init --merge-devel --cmake-args -DCMAKE_BUILD_TYPE=Release
 catkin config --extend /opt/ros/melodic
 sudo apt-get install ros-melodic-rosbridge-server
 cd ~/catkin_ws/src
 sudo apt-get install ros-$ROS_DISTRO-catkin ros-$ROS_DISTRO-tf2-eigen python-catkin-tools libgflags-dev libgoogle-glog-dev python-wstool
 git clone https://github.com/EricVoll/vio_drift_comp_using_ASA mr-drone
 git clone https://github.com/ethz-asl/rovio 
 cd rovio && git submodule update --init --recursive && cd ..
 git clone -b feature/poseupdate https://github.com/ethz-asl/rovio
 git clone https://github.com/catkin/catkin_simple.git
 git clone https://github.com/ethz-asl/kindr
 git clone https://github.com/ethz-asl/rotors_simulator
 git clone https://github.com/ethz-asl/mav_comm
 sudo apt-get update
 sudo apt-get install ros-melodic-mavros ros-melodic-mavros-extras
 sudo apt-get install ros-melodic-octomap ros-melodic-octomap-ros protobuf-compiler
 
 catkin build
 ```
 If you run into weird errors while building, try a clean build by deleting the `catkin_ws/build` folder.
 
 If you want to run the sensorPod setup or the real drone, you will have to install versavis, rosserial, flircameradriver.
 
## Catkin folder
My working catkin folder had the following packages in it. Probably not all of them have to be catkinized versions, some are not necesarry for this project.

<details>
  <summary>Click to expand!</summary>

- azure_spatial_anchors_ros      
- cgal_catkin          
- gflags_catkin    
- json_catkin      
- mav_comm      
- minkindr_gtsam
- rovio
- cad-percept                    
- eigen_catkin         
- glog_catkin      
- kindr            
- mav_tools     
- minkindr_ros    
- rpg_trajectory_evaluation
- catkin_boost_python_buildtool  
- eigen_checks         
- gtsam_catkin     
- libnabo          
- metis_catkin  
- mr-drone
- catkin_simple                  
- ethzasl_icp_mapping  
- image_undistort  
- libpointmatcher  
- minkindr      
- numpy_eigen
</details>

 
# Depenencies:
- https://github.com/ethz-asl/rotors_simulator
- https://github.com/EricVoll/ros-sharp
- https://github.com/ethz-asl/rovio (read this [issue](https://github.com/ethz-asl/rovio/issues/183) for installing)

# Configuration:
The following things will have to be configured:
 - `asa_ros.launch` [link](https://github.com/EricVoll/SemesterProjectROS/blob/main/launch/asa_ros.launch): Valid account details have to be inserted. You can follow the description [here](https://github.com/microsoft/azure_spatial_anchors_ros/wiki#requirements). If the `camera_info` and `image` topic are not perfectly synchronized (down to the ns) activate the approximate time sync policy in the launch file. Otherwise ASA won't work.
 - `Rovio.info` [link](https://github.com/EricVoll/SemesterProjectROS/blob/main/cfg/rovio.info): This file is required for rovio to work. It is more or less self-explaining.
 - `Camera.yaml` [link](https://github.com/EricVoll/SemesterProjectROS/blob/main/cfg/cam0.yaml): This is a camera intrinsics configuration file, required for versavis and rovio.
 - `runs.yaml` [link](https://github.com/EricVoll/SemesterProjectROS/blob/main/cfg/runs.yaml): This file is the config file of the test-automation written for this project. The pipeline reads this config file, executes all tests, aligns the trajectories (ground truth + estimate) and even generates nice plots for you. There is a bunch of options available, each and everyong should be explained using comments in the yaml file.

# Overview

There are many different components to this project. I will first describe some of the source files, their responsibilities and how to use them.
|Source File|Link|Description|
|-|-|-|
|asa_handler|[link](https://github.com/EricVoll/SemesterProjectROS/blob/main/src/asa_handler.py)|The asa handler is a python class that manages asa. There are different parameters in the constructor. The parameters and their meaning is defined as comments in the python file. This class can spin up asa_nodes, can either wait for enough data to be accumulated & create an anchor, or can take an anchor id and start to look for it. If configured, it will reset the asa_node after finding an anchor and restart the process. (Side info: if an anchor was created by a HoloLens then it has to be rotated for ROS. This is why its important to call `asa_handler.SetIsHoloLensAnchor(True/False)`. If True, the anchor will be rotated and republished. Otherwise this won't happen.
|drift_compensator|[link](https://github.com/EricVoll/SemesterProjectROS/blob/main/src/drift_compensator.py)|This is the main file of this repo. It uses asa_handlers to constantly query anchors and estimates rovio's drift. It can the either send odometry updates to rovio or publish a drift-compensated tf. The three most important paramters are: `anchor_id_preset`: An ASA anchor that is used for the drift compensation instead of creating a new anchor. The anchor id is passed on to one of the asa_handlers. `covariance_mode`: Can either be "empirical", or a floating point number. If "empirical" is used, the drift compensator calculates the empirical covariance of an asa_handler's observations and uses it for the pose updates. If a float number is used, then the covariance is set to this number for all entries on the main diagonal of the covariance. The last important parameter is `send_update`, which has two three modes: 0: Do not send updates to rovio at all. The node will instead publish a "imu_alt" tf every frame. 1: Send updates to rovio until rovio's world frame starts to drift (world frame drift is published under `rovio/T_G_W`. The node then has to stop sending updates, because it would otherwise result in a positive feedback loop. 2: Only send updates to rovio using every anchor observation once. The last important parameter is `multiple_anchors`, which can be True or False. If true, the node will spin up multiple asa_handlers and observe multiple anchors at the same time.
|synchronizer|[link](https://github.com/EricVoll/SemesterProjectROS/blob/main/src/synchronizer.py)|This node subscribes to a camera_info and Image topic with an approximate time synch policy and republishes them with the same timestep. 
|statistics|[link](https://github.com/EricVoll/SemesterProjectROS/blob/main/src/statistics.py)|A helper class that collects anchor observations and can calculate the coriance as derived in the report. It also able to calculate the pose update, but the current implementation just uses TF instead of doing it manually.
|rovio_world_drift_observer|[link](https://github.com/EricVoll/SemesterProjectROS/blob/main/src/rovio_world_drift_observer.py)|A class that listens to Rovio's world drift topic and calls a callback if it exceeds a threshold drift velocity.
|runner & analyzer|[link](https://github.com/EricVoll/SemesterProjectROS/blob/main/src/runner.py)|This script reads the `runs.yaml` file and creates a dictionary of all configured runs. If the runs.yaml file says it to do so, it will then start the analyzer that generates statistics and plots.
|stability_analyzer & analyze_anchor_precision|[link](https://github.com/EricVoll/SemesterProjectROS/blob/main/src/stability_analyzer.py)|The stability analyzer takes an anchor id and lets ASA query the anchor constantly (using an asa_handler) while recording all the returned transforms. It then writes the anchor timestamps and transforms to csv files.

# Reproducing the test results
There are a two things from the report that can be reproduced

## Reproducing ASA precision/robustnes results
First, the anchor ID to be used has to be changed. It is not imortant which device created the anchor. Do that by changing the hardcoded id in [this file](https://github.com/EricVoll/SemesterProjectROS/blob/main/src/stability_analyzer.py). The output file paths at the bottom will have to be replaced as well.

Execute the test by 
Terminal 1: Start a roscore: `$ roscore`
Terminal 2: Active sim time: `$ rosparam set use_sim_time true`
Terminal 2: Start the image pre-processing (recitifying etc.): `$roslaunch mr-drone image_preprocessing.launch`
Terminal 3: Start the analyzer node `$ rosrun mr-drone stability_analyzer`
Terminal 4: Start the rosbag: `$ rosbag play --clock bag1.bag`

If you close the node in terminal 3 it will save the arrays to the disk.
You can then run `$ cd catkin_ws/src/mr-drone/src && python analyze_anchor_precision.py`, which will read this file and return a few stats.

## Reproducing VIO drift compensation performance results
If valid config files (/cfg/runs.yaml) exist, this should be as easy as doing: 
`$ cd catkin_ws/src/mr-drone/src && python runner.py`
The structure and available options of the yaml file are documented in the included yaml file as comments.
