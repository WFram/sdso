# SDSO: Stereo Direct Sparse Odometry 

Stereo Direct Sparse Odometry package, which supports ROS interface. Mostly designed for mapping purposes upon [DSO](https://github.com/JakobEngel/dso)
and [Stereo DSO](https://github.com/JiatianWu/stereo-dso) libraries.

### **Related Papers:** 

* **Direct Sparse Odometry**, *J. Engel, V. Koltun, D. Cremers*, In arXiv:1607.02565, 2016
* **Large-scale direct SLAM with stereo cameras**, *J. Engel, J. Stückler, D. Cremers*, IROS, 2015
### Installation
The package requires pre-installed libraries that are the same as for [Stereo DSO](https://github.com/JiatianWu/stereo-dso).
Also, PCL library is required for use of ROS interface.

```
mkdir ~/stereo_dso_ws/src
cd ~/stereo_dso_ws/src
git clone git@github.com:WFram/sdso.git
# In case of using ARM
# git submodule update --init
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