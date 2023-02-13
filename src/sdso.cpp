//
// Created by wfram on 2/13/23.
//

#include "sdso.h"

SDSO::SDSO()
    : useSampleOutput(false),
      preload(false),
      mode(0),
      playbackSpeed(0),
      frameID(0),
      stopSystem(false),
      start(2),
      calib(""),
      vignetteFile(""),
      gammaFile("") {
  parseSettings();

  undistorter.reset(Undistort::getUndistorterForFile(calib, gammaFile, vignetteFile));

  // TODO: use static casting
  setGlobalCalib((int)undistorter->getSize()[0], (int)undistorter->getSize()[1], undistorter->getK().cast<float>());
  baseline = undistorter->getBl();

  if (!disableAllDisplay)
    viewer = new IOWrap::PangolinDSOViewer((int)undistorter->getSize()[0], (int)undistorter->getSize()[1]);

  fullSystem = std::make_unique<FullSystem>();
  fullSystem->linearizeOperation = false;

  if (undistorter->photometricUndist != nullptr) fullSystem->setGammaFunction(undistorter->photometricUndist->getG());

  if (viewer) fullSystem->outputWrapper.push_back(viewer);

  if (useSampleOutput) fullSystem->outputWrapper.push_back(new IOWrap::SampleOutputWrapper());

  rosOutput = new ROSOutputWrapper();
  fullSystem->outputWrapper.push_back(rosOutput);
}

void SDSO::parseSettings() {
  ros::param::get("playbackSpeed", playbackSpeed);
  ros::param::get("preload", preload);

  ros::param::get("mode", mode);
  ros::param::get("sampleoutput", sampleoutput);
  ros::param::get("quiet", quiet);
  ros::param::get("nolog", nolog);
  ros::param::get("nogui", nogui);
  ros::param::get("nomt", nomt);
  ros::param::get("calib", calib);
  ros::param::get("left_image_topic", left_image_topic_);
  ros::param::get("right_image_topic", right_image_topic_);

  ros::param::get("setting_desiredImmatureDensity", setting_desiredImmatureDensity);
  ros::param::get("setting_desiredPointDensity", setting_desiredPointDensity);
  ros::param::get("setting_minFrames", setting_minFrames);
  ros::param::get("setting_maxFrames", setting_maxFrames);
  ros::param::get("setting_maxOptIterations", setting_maxOptIterations);
  ros::param::get("setting_minOptIterations", setting_minOptIterations);
  ros::param::get("setting_kfGlobalWeight", setting_kfGlobalWeight);
  ros::param::get("setting_maxShiftWeightT", setting_maxShiftWeightT);
  ros::param::get("setting_maxShiftWeightR", setting_maxShiftWeightR);
  ros::param::get("setting_maxShiftWeightRT", setting_maxShiftWeightRT);
  ros::param::get("setting_logStuff", setting_logStuff);

  switch (mode) {
    case 0:
      ROS_INFO("PHOTOMETRIC MODE WITH CALIBRATION!\n");
    case 1:
      ROS_INFO("PHOTOMETRIC MODE WITHOUT CALIBRATION!\n");
      setting_photometricCalibration = 0;
      setting_affineOptModeA = 0;  //-1: fix. >=0: optimize (with prior, if > 0).
      setting_affineOptModeB = 0;  //-1: fix. >=0: optimize (with prior, if > 0).
    case 2:
      printf("PHOTOMETRIC MODE WITH PERFECT IMAGES!\n");
      setting_photometricCalibration = 0;
      setting_affineOptModeA = -1;  //-1: fix. >=0: optimize (with prior, if > 0).
      setting_affineOptModeB = -1;  //-1: fix. >=0: optimize (with prior, if > 0).
      setting_minGradHistAdd = 3;
  }

  if (sampleoutput == 1) {
    useSampleOutput = true;
    ROS_INFO("USING SAMPLE OUTPUT WRAPPER!\n");
  }

  if (quiet == 1) {
    setting_debugout_runquiet = true;
    printf("QUIET MODE, I'll shut up!\n");
  }

  if (nolog == 1) {
    setting_logStuff = false;
    printf("DISABLE LOGGING!\n");
  }

  if (nogui == 1) {
    disableAllDisplay = true;
    printf("NO GUI!\n");
  }

  if (nomt == 1) {
    multiThreading = false;
    printf("NO MultiThreading!\n");
  }

  printf("loading calibration from %s!\n", calib.c_str());
  printf("loading vignette from %s!\n", vignetteFile.c_str());
  printf("loading gammaCalib from %s!\n", gammaFile.c_str());
}

void SDSO::callback(const sensor_msgs::ImageConstPtr &img_left, const sensor_msgs::ImageConstPtr &img_right) {
  auto convertStamp = [](const ros::Time &time) { return time.sec * 1.0 + time.nsec / 1000000000.0; };

  double stamp = convertStamp(img_left->header.stamp);

  cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(img_left, sensor_msgs::image_encodings::MONO8);
  assert(cv_ptr->image.type() == CV_8U);
  assert(cv_ptr->image.channels() == 1);

  cv_bridge::CvImagePtr cv_ptr_right = cv_bridge::toCvCopy(img_right, sensor_msgs::image_encodings::MONO8);
  assert(cv_ptr_right->image.type() == CV_8U);
  assert(cv_ptr_right->image.channels() == 1);

  if (fullSystem->initFailed || setting_fullResetRequested) {
    printf("RESETTING!\n");
    std::vector<IOWrap::Output3DWrapper *> wraps = fullSystem->outputWrapper;
    fullSystem.reset();
    for (IOWrap::Output3DWrapper *ow : wraps) ow->reset();

    fullSystem = std::make_unique<FullSystem>();
    fullSystem->linearizeOperation = false;
    fullSystem->outputWrapper = wraps;
    if (undistorter->photometricUndist != 0) fullSystem->setGammaFunction(undistorter->photometricUndist->getG());
    setting_fullResetRequested = false;
  }

  if (fullSystem->isLost) {
    printf("LOST!!\n");
    ros::shutdown();
  }

  rosOutput->publishOutput();

  MinimalImageB minImg((int)cv_ptr->image.cols, (int)cv_ptr->image.rows, (unsigned char *)cv_ptr->image.data);
  MinimalImageB minImg_right((int)cv_ptr_right->image.cols, (int)cv_ptr_right->image.rows,
                             (unsigned char *)cv_ptr_right->image.data);
  ImageAndExposure *undistImg_left = undistorter->undistort<unsigned char>(&minImg, 1, stamp, 1.0f);
  ImageAndExposure *undistImg_right = undistorter->undistort<unsigned char>(&minImg_right, 1, stamp, 1.0f);

  // TODO: here we should add images
  fullSystem->addActiveFrame(undistImg_left, undistImg_right, frameID);
  frameID++;
  delete undistImg_left;
  delete undistImg_right;

  if (stopSystem) {
    fullSystem->blockUntilMappingIsFinished();

    for (IOWrap::Output3DWrapper *ow : fullSystem->outputWrapper) ow->join();

    printf("DELETE FULLSYSTEM!\n");
    undistorter.reset();
    fullSystem.reset();

    ros::shutdown();

    printf("EXIT NOW!\n");
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "sdso");
  ros::NodeHandle nh;

  setlocale(LC_ALL, "C");

  auto sdso_system = std::make_shared<SDSO>();

  message_filters::Subscriber<sensor_msgs::Image> left_sub(nh, "/cam0/image_raw", 1);
  message_filters::Subscriber<sensor_msgs::Image> right_sub(nh, "/cam1/image_raw", 1);
  // TODO: change to alias
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;
  message_filters::Synchronizer<sync_pol> sync(sync_pol(10), left_sub, right_sub);
  // TODO: check if there will not be an issue with memory management after we finished
  sync.registerCallback(boost::bind(&SDSO::callback, std::move(sdso_system), _1, _2));

  ros::spin();
  sdso_system->stop();

  return 0;
}
