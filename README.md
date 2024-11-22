# ORB-SLAM3-and-BD-Spot
This was for a final project for EECE-5554: Robotics Sensing &amp; Navigation at Northeastern University. We implemented and used ORB_SLAM3 to perform Visual SLAM on a variety of data we collected using the Boston Dynamics Spot dog. This will explain how to run ORB-SLAM3 and load the datasets

# ORB-SLAM-3 Setup
Prerequisites:
- Ubuntu 22.04
- ROS2 Humble

Follow the instructions to download ORB-SLAM3 at the following link: https://github.com/Mechazo11/ros2_orb_slam3

## Download sample datasets
The ORB-SLAM repository provides sample data from EuRoC MH_05 however, to view other datasets (such as MH_01), we can do the following:

    cd ~
    mkdir -p sample_data/EuRoc
    cd sample_data/EuRoc/
    wget -c http://robotics.ethz.ch/~asl-datasets/ijrr_euroc_mav_dataset/machine_hall/MH_01_easy/MH_01_easy.zip
    mkdir MH01
    unzip MH_01_easy.zip -d MH01/


Once this dataset is downloaded, you can copy it into your ORB-SLAM3 workspace

# Estimates vs Ground Truth Analysis

# Data Collection on Spot
Since the data collected from Spot gives us a rosbag, we will need to convert the rosbag so that we can extract the image frames from the dataset.

We will also need the calibration parameters from the Spot cameras

Once we extracted the images from the dataset we collected on Spot, we were able to copy the images into the ORB-SLAM3 workspace and run ORB-SLAM3.

# References
https://github.com/Mechazo11/ros2_orb_slam3
https://github.com/bdaiinstitute/spot_ros2
https://github.com/UZ-SLAMLab/ORB_SLAM3
