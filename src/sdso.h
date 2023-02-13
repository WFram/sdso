//
// Created by wfram on 2/13/23.
//

#include <locale.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include "FullSystem.h"
#include "OutputWrapper/SampleOutputWrapper.h"
#include "Pangolin/PangolinDSOViewer.h"
#include "Undistort.h"
#include "settings.h"

#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include "cv_bridge/cv_bridge.h"

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>

#include "DatasetReader.h"

#include "ROSWrapper/ROSOutputWrapper.h"

using namespace dso;

#ifndef SDSO_SDSO_H
#define SDSO_SDSO_H

class SDSO {
 public:
  explicit SDSO();

  void parseSettings();

  void callback(const sensor_msgs::ImageConstPtr &img_left, const sensor_msgs::ImageConstPtr &img_right);

  void stop() { stopSystem = true; }

 private:
  std::unique_ptr<FullSystem> fullSystem;
  std::unique_ptr<Undistort> undistorter;
  IOWrap::PangolinDSOViewer *viewer;
  dso::ROSOutputWrapper *rosOutput;

  int frameID;
  bool stopSystem;
  int start;

  int mode, sampleoutput, quiet, nolog, nogui, nomt;
  std::string calib, vignetteFile, gammaFile;

  std::string left_image_topic_, right_image_topic_;

  bool useSampleOutput;
  bool preload;
  float playbackSpeed;  // 0 for linearize (play as fast as possible, while sequentializing tracking & mapping).
                        // otherwise, factor on timestamps.
};

#endif  // SDSO_SDSO_H
