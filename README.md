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
# Create a workspace (really better to use catkin tools)
mkdir -p ~/workspace/src
catkin init
catkin config --extend /opt/ros/noetic
catkin config --merge-devel
catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_VERBOSE_MAKEFILE:BOOL=ON

# Build cv_bridge from source
cd ~/workspace/src
git clone git@github.com:WFram/cv_bridge_local.git
cd ~/workspace
catkin build cv_bridge

# Build sdso
cd ~/workspace/src
git clone -b devel/cv_bridge_deps git@github.com:WFram/sdso.git

# In case of using ARM
# cd ~/workspace/src/sdso
# git submodule update --init

cd ~/workspace
catkin build sdso

# Run
source devel/setup.bash
roslaunch sdso zed2_odometry.launch use_rviz:=true
```

### Configuring

Topics for stereo images and path to camera calibration settings can be set in `.launch` files.
Several options for output point cloud filtering are available to configure in `common.yaml`.