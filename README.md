# SDSO: Stereo Direct Sparse Odometry 

---
Stereo DSO package, which supports ROS interface. Mostly designed for mapping purposes upon [DSO](https://github.com/JakobEngel/dso)
and [Stereo DSO](https://github.com/JiatianWu/stereo-dso) libraries.

### **Related Papers:** 

* **Direct Sparse Odometry**, *J. Engel, V. Koltun, D. Cremers*, In arXiv:1607.02565, 2016
* **Large-scale direct SLAM with stereo cameras**, *J. Engel, J. Stückler, D. Cremers*, IROS, 2015
### Installation

```
mkdir ~/stereo_dso_ws/src
cd ~/stereo_dso_ws/src
git clone git@github.com:WFram/sdso.git
cd ~/stereo_dso_ws
catkin build sdso
source ~/stereo_dso_ws/devel/setup.bash
```

### Usage

```
roslaunch sdso zed2_odometry.launch
```

### Configuring

Topics for stereo images and path to camera calibration settings can be set in `.launch` files.
Several options for output point cloud filtering are available to configure in `common.yaml`.