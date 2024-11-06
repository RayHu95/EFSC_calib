# A Dynamic Calibration Framework for Event-frame Stereo Camera (EFSC) System

<a href="http://www.youtube.com/watch?feature=player_embedded&v=LCYqNZBBT6s" target="_blank">
  <img src = "/pic/workflow.jpg" width = 100%  alt="The calibration diagram for the EFSC system." >
</a>

**Abstract**: Our paper develops a dynamic calibration framework for the event-frame stereo camera system. ... 
In this framework, the first step is to complete the initial detection on a **circle-grid calibration pattern**, and **a sliding-window time matching method** is proposed to match the event-frame pairs. Then, a
refining method is devised for two cameras to get the accurate information of the pattern. Particularly, for the event camera, a
**patch-size motion compensation method** is designed to achieve time synchronization for two cameras and fit circles in an image of warped events. Finally, the
pose between two cameras is globally optimized by constructing **a pose-landmark graph** with two types of edges.


**Testing Dataset**: It contains two testing setups: “parallel” setup and “hand-recorded” setup. The details can be found in the paper.

## Test requirement
- Ubuntu 18.04
- ROS Melodic
- dv-processing 1.4.0
- OpenCV 4.0
- nanoflann
- Eigen3
- Ceres
- Sophus
- G2O

The prototype of the EFSC system contains a classical [DAVIS346](https://dv-processing.inivation.com/rel_1_7/installation.html) camera with 346X260 pixel resolution and an 848X480 color camera module of Intel RealSense [D435i](https://github.com/IntelRealSense/realsense-ros). 


## roslaunch the code
```
roslaunch ef_ros_calib calib_node_bag.launch
```
Note that the **path** of the dataset should be changed.

## Citations

The code is for research purpose. Please cite the following:

```
@ARTICLE{CalibEFSC,
  author={Hu, Rui and Kogler, J{\"u}rgen and Gelautz, Margrit and Lin, Min and Xia, Yuanqing},
  journal={IEEE Robotics and Automation Letters}, 
  title={A Dynamic Calibration Framework for the Event-Frame Stereo Camera System}, 
  year={2024},
  volume={},
  number={},
  pages={1-8},
  doi={10.1109/LRA.2024.3491426}}
```
TO BE CONTINUE ...
