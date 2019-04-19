# ORB_SLAM2_Map
Real-time envrionment reconstruction based on ORB_SLAM2 with XTION (RGBD sensor) <br>
 
### Introduction
This is a modified version based on [ORB_SLAM2](https://github.com/raulmur/ORB_SLAM2) and [ORBSLAM2_with_pointcloud_map](https://github.com/gaoxiang12/ORBSLAM2_with_pointcloud_map) , thanks for Raul's and Gao's great work! This works with XTION whicn privide us the rgb image and depth image and reconstruct the around environment by the trajectory of sensors. You can visualize the environment by point cloud map during the SLAM process. <br>

### How to install
Unzip the file orb_slam_rgbd.zip, you will find two directories. First compile the modified g2o: <br>
```c
  cd g2o_with_orbslam2
  mkdir build
  cd build
  cmake ..
  make 
```
If you have some problems, follow the instructions from the [original g2o library](https://github.com/RainerKuemmerle/g2o) <br>

Then compile the ORB_SLAM2. You need firstly to compile the DBoW2 in ORB_SLAM2_modified/Thirdpary, and then the [Pangolin module](https://github.com/stevenlovegrove/Pangolin). Finally, build ORB_SLAM2:
```c
  cd ORB_SLAM2_modified
  mkdir build
  cd build
  cmake ..
  make
```
Due to the limited size, the vocabulary file is not contained in the zip file. Please download that .txt file from [ORB_SLAM2_Vocabulary](https://github.com/raulmur/ORB_SLAM2/tree/master/Vocabulary) and put it in Vocabulary/.

### Run examples
Then you can run the examples. In the **Examples/RGB-D**, **rgbd_tum** deals with the datasets of TUM (**desk.png room.png**), **rgbd_cc** deals with the datasets of your own, **rgbd_xtion_cc** runs with XTION and reconstruct the environment in real-time (**orb_rgbd_slam.png lab_1-3.png**). To run **rgbd_xtion_cc**, you can try under ORB_SLAM2_modified/: <br>
```c
  ./Examples/RGB-D/rgbd_xtion_cc Vocabulary/ORBvoc.txt ./Examples/RGB-D/xtion.yaml
```
You can get the intrinsic parameters of your xtion by ROS/OPENCV/MATLAB **calibration**. <br>
 
### Notes
1. If you use **kinectv1**, just change the driver for kinect and this project could work. <br>
2. If you use **kinectv2**, maybe you need to change the driver and code to get data from sensors. <br>
3. If you have some problems about g2o when compiling the orb_slam2, it must be the problem that orb_slam2 finds a wrong version of g2o. Change the **CMakeLists.txt** and **add absolute path** for g2o which is the g2o_with_orbslam2 you just compiled.

[源代码地址](https://github.com/chaizheng2157/RGBD_ORB_SLAM2_RT.git) 

A real-time reconstruction program with Xtion based on orb_slam2.
More details: http://blog.csdn.net/aptx704610875/article/details/51490201

run 

	  ../Examples/RGB-D/rgbd_xtion_cc ../Vocabulary/ORBvoc.txt ../Examples/RGB-D/xtion.yaml

	../Examples/RGB-D/rgbd_xtion_cc ../Vocabulary/ORBvoc.bin ../Examples/RGB-D/xtion.yaml

### Tools
txt词袋转二进制词袋

 	./tools/bin_vocabulary
 	
git add --all
git commit -m "first commit"
git push origin master

新建
git init
git add .
git commit -m "first commit"
git remote rm origin
git remote add origin https://github.com/ardupilotdrone/ardupilot.git
git push -u origin +master 

git pull 合并分支