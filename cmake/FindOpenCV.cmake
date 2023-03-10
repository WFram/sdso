# NOTE:
#       for catkin build : ${CMAKE_BINARY_DIR}/../cv_bridge/3rd_party/src/opencv_external-build/install/include/opencv4/opencv2
#       for catkin_make : ${CMAKE_BINARY_DIR}/cv_bridge_local/3rd_party/src/opencv_external-build/install/include/opencv4/opencv2

FIND_PATH(opencv_INCLUDE_DIR NAMES opencv.hpp
        PATHS
        ${CMAKE_BINARY_DIR}/../cv_bridge/3rd_party/src/opencv_external-build/install/include/opencv4/opencv2
        )

message("Searching for Local OpenCV in ${CMAKE_BINARY_DIR}/../cv_bridge/3rd_party/src/opencv_external-build/install/include/opencv4/opencv2")
message("opencv_INCLUDE_DIR: ${opencv_INCLUDE_DIR}")

# TODO: make sure it's scalable
set(CMAKE_FIND_ROOT_PATH "")
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)

FIND_LIBRARY(opencv_core_LIB NAMES opencv_core
        PATHS
        ${CMAKE_BINARY_DIR}/../cv_bridge/3rd_party/src/opencv_external-build/install/lib
        NO_DEFAULT_PATH
        )

FIND_LIBRARY(opencv_flann_LIB NAMES opencv_flann
        PATHS
        ${CMAKE_BINARY_DIR}/../cv_bridge/3rd_party/src/opencv_external-build/install/lib
        NO_DEFAULT_PATH
        )

FIND_LIBRARY(opencv_imgproc_LIB NAMES opencv_imgproc
        PATHS
        ${CMAKE_BINARY_DIR}/../cv_bridge/3rd_party/src/opencv_external-build/install/lib
        NO_DEFAULT_PATH
        )

FIND_LIBRARY(opencv_features2d_LIB NAMES opencv_features2d
        PATHS
        ${CMAKE_BINARY_DIR}/../cv_bridge/3rd_party/src/opencv_external-build/install/lib
        NO_DEFAULT_PATH
        )

FIND_LIBRARY(opencv_imgcodecs_LIB NAMES opencv_imgcodecs
        PATHS
        ${CMAKE_BINARY_DIR}/../cv_bridge/3rd_party/src/opencv_external-build/install/lib
        NO_DEFAULT_PATH
        )

FIND_LIBRARY(opencv_videoio_LIB NAMES opencv_videoio
        PATHS
        ${CMAKE_BINARY_DIR}/../cv_bridge/3rd_party/src/opencv_external-build/install/lib
        NO_DEFAULT_PATH
        )

FIND_LIBRARY(opencv_calib3d_LIB NAMES opencv_calib3d
        PATHS
        ${CMAKE_BINARY_DIR}/../cv_bridge/3rd_party/src/opencv_external-build/install/lib
        NO_DEFAULT_PATH
        )

FIND_LIBRARY(opencv_highgui_LIB NAMES opencv_highgui
        PATHS
        ${CMAKE_BINARY_DIR}/../cv_bridge/3rd_party/src/opencv_external-build/install/lib
        NO_DEFAULT_PATH
        )

FIND_LIBRARY(opencv_video_LIB NAMES opencv_video
        PATHS
        ${CMAKE_BINARY_DIR}/../cv_bridge/3rd_party/src/opencv_external-build/install/lib
        NO_DEFAULT_PATH
        )

FIND_LIBRARY(opencv_xfeatures2d_LIB NAMES opencv_xfeatures2d
        PATHS
        ${CMAKE_BINARY_DIR}/../cv_bridge/3rd_party/src/opencv_external-build/install/lib
        NO_DEFAULT_PATH
        )

SET(OPENCV_LIBS
        ${opencv_core_LIB}
        ${opencv_flann_LIB}
        ${opencv_imgproc_LIB}
        ${opencv_features2d_LIB}
        ${opencv_imgcodecs_LIB}
        ${opencv_videoio_LIB}
        ${opencv_calib3d_LIB}
        ${opencv_highgui_LIB}
        ${opencv_video_LIB}
        ${opencv_xfeatures2d_LIB}
        )

message("OPENCV_LIBS: ${OPENCV_LIBS}")

IF (opencv_INCLUDE_DIR AND OPENCV_LIBS)
    SET(opencv_FOUND TRUE)
ELSE (opencv_INCLUDE_DIR AND OPENCV_LIBS)
    SET(opencv_FOUND FALSE)
ENDIF (opencv_INCLUDE_DIR AND OPENCV_LIBS)
