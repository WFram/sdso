SET(PROJECT_NAME sdso C CXX)

PROJECT(${PROJECT_NAME})
cmake_minimum_required(VERSION 3.16)

set(BUILD_TYPE RelWithDebInfo)

include(cmake/options.cmake)

set(EXECUTABLE_OUTPUT_PATH bin)
set(LIBRARY_OUTPUT_PATH lib)
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

find_package(SuiteParse REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost COMPONENTS system thread)

find_package(OpenCV REQUIRED)

if (opencv_FOUND)
    message("Found local OpenCV")
else ()
    message(FATAL_ERROR "Local OpenCV NOT found")
endif ()

find_package(LibZip QUIET)
find_package(Pangolin 0.2 QUIET)

# flags
add_definitions("-DENABLE_SSE")
set(CMAKE_CXX_FLAGS
        "${SSE_FLAGS}"
        )

# Sources files
set(sdso_SOURCE_FILES
        ${PROJECT_SOURCE_DIR}/src/FullSystem/FullSystem.cpp
        ${PROJECT_SOURCE_DIR}/src/FullSystem/FullSystemOptimize.cpp
        ${PROJECT_SOURCE_DIR}/src/FullSystem/FullSystemOptPoint.cpp
        ${PROJECT_SOURCE_DIR}/src/FullSystem/FullSystemDebugStuff.cpp
        ${PROJECT_SOURCE_DIR}/src/FullSystem/FullSystemMarginalize.cpp
        ${PROJECT_SOURCE_DIR}/src/FullSystem/Residuals.cpp
        ${PROJECT_SOURCE_DIR}/src/FullSystem/CoarseTracker.cpp
        ${PROJECT_SOURCE_DIR}/src/FullSystem/CoarseInitializer.cpp
        ${PROJECT_SOURCE_DIR}/src/FullSystem/ImmaturePoint.cpp
        ${PROJECT_SOURCE_DIR}/src/FullSystem/HessianBlocks.cpp
        ${PROJECT_SOURCE_DIR}/src/FullSystem/PixelSelector2.cpp
        ${PROJECT_SOURCE_DIR}/src/OptimizationBackend/EnergyFunctional.cpp
        ${PROJECT_SOURCE_DIR}/src/OptimizationBackend/AccumulatedTopHessian.cpp
        ${PROJECT_SOURCE_DIR}/src/OptimizationBackend/AccumulatedSCHessian.cpp
        ${PROJECT_SOURCE_DIR}/src/OptimizationBackend/EnergyFunctionalStructs.cpp
        ${PROJECT_SOURCE_DIR}/src/util/settings.cpp
        ${PROJECT_SOURCE_DIR}/src/util/Undistort.cpp
        ${PROJECT_SOURCE_DIR}/src/util/globalCalib.cpp
        ${PROJECT_SOURCE_DIR}/src/IOWrapper/OpenCV/ImageDisplay_OpenCV.cpp
        ${PROJECT_SOURCE_DIR}/src/IOWrapper/OpenCV/ImageRW_OpenCV.cpp
        )

include_directories(
        ${PROJECT_SOURCE_DIR}/src/FullSystem
        ${PROJECT_SOURCE_DIR}/src/IOWrapper
        ${PROJECT_SOURCE_DIR}/src/OptimizationBackend
        ${PROJECT_SOURCE_DIR}/src/util
        ${PROJECT_SOURCE_DIR}/thirdparty/Sophus
        ${PROJECT_SOURCE_DIR}/thirdparty/sse2neon
        ${opencv_INCLUDE_DIR}/../ # /path/to/opencv4
        ${EIGEN3_INCLUDE_DIR}
)

# decide if we have pangolin
if (Pangolin_FOUND)
    message("--- found PANGOLIN, compiling sdso_pangolin library.")
    include_directories(${Pangolin_INCLUDE_DIRS})
    set(sdso_pangolin_SOURCE_FILES
            ${PROJECT_SOURCE_DIR}/src/IOWrapper/Pangolin/KeyFrameDisplay.cpp
            ${PROJECT_SOURCE_DIR}/src/IOWrapper/Pangolin/PangolinDSOViewer.cpp)
    set(HAS_PANGOLIN 1)
else ()
    message("--- could not find PANGOLIN, not compiling sdso_pangolin library.")
    message("    this means there will be no 3D display / GUI available for sdso_dataset.")
    set(sdso_pangolin_SOURCE_FILES)
    set(HAS_PANGOLIN 0)
endif ()

# decide if we have ziplib.
if (LIBZIP_LIBRARY)
    message("--- found ziplib (${LIBZIP_VERSION}), compiling with zip capability.")
    add_definitions(-DHAS_ZIPLIB=1)
    include_directories(${LIBZIP_INCLUDE_DIR_ZIP} ${LIBZIP_INCLUDE_DIR_ZIPCONF})
else ()
    message("--- not found ziplib (${LIBZIP_LIBRARY}), compiling without zip capability.")
    set(LIBZIP_LIBRARY "")
endif ()

# compile main library.
include_directories(${CSPARSE_INCLUDE_DIR} ${CHOLMOD_INCLUDE_DIR})
add_library(sdso ${sdso_SOURCE_FILES} ${sdso_pangolin_SOURCE_FILES})

target_link_libraries(sdso PUBLIC ${OPENCV_LIBS})

# build main executable (only if we have both OpenCV and Pangolin)
if (Pangolin_FOUND)
    message("--- compiling sdso_dataset.")
    add_executable(sdso_dataset ${PROJECT_SOURCE_DIR}/src/main_dso_pangolin.cpp)
    target_link_libraries(sdso_dataset sdso boost_system boost_thread cxsparse ${LIBZIP_LIBRARY} ${Pangolin_LIBRARIES})
else ()
    message("--- not building sdso_dataset, since either don't have openCV or Pangolin.")
endif ()

# NOTE: if there is a need to build with ROS, use catkin build only
if (BUILD_WITH_ROS)
    message("Building as a ROS package")
    find_package(catkin REQUIRED COMPONENTS
            geometry_msgs
            roscpp
            sensor_msgs
            tf2
            tf2_ros
            pcl_conversions
            message_filters
            )
    find_package(PCL 1.8 REQUIRED)
    # TODO: think how to build locally
    find_package(Boost COMPONENTS system thread filesystem chrono serialization date_time timer regex)
    catkin_package(
            CATKIN_DEPENDS
            geometry_msgs
            tf2
            tf2_ros
            pcl_conversions
            roscpp
            sensor_msgs
            message_filters
    )
    # TODO: set using predefined var
    set(cv_bridge_INCLUDE_DIRS "${CATKIN_DEVEL_PREFIX}/../src/cv_bridge_local/include")
    include_directories(
            ${PROJECT_SOURCE_DIR}/src/ROSWrapper
            ${PROJECT_SOURCE_DIR}/src/
            ${PCL_INCLUDE_DIRS}
            ${cv_bridge_INCLUDE_DIRS}
            ${catkin_INCLUDE_DIRS}
    )
    message("cv_bridge_INCLUDE_DIRS: ${cv_bridge_INCLUDE_DIRS}")
    add_compile_definitions(CONFIG_DIR="${PROJECT_SOURCE_DIR}/config/")
    set(CONFIG_DIR "${PROJECT_SOURCE_DIR}/config/")
    add_executable(sdso_ros src/sdso.cpp src/ROSWrapper/ROSOutputWrapper.cpp)
    add_library(sdso_ros_setup src/empty.cpp)
    set(cv_bridge_LIBRARIES ${CATKIN_DEVEL_PREFIX}/lib/libcv_bridge.so)
    target_link_libraries(sdso_ros_setup
            ${BOOST_THREAD_LIBRARY}
            ${PCL_LIBRARIES}
            ${cv_bridge_LIBRARIES}
            ${catkin_LIBRARIES}
            )
    message("cv_bridge_LIBRARIES: ${cv_bridge_LIBRARIES}")
    link_directories(${CATKIN_DEVEL_PREFIX}/lib NO_DEFAULT_PATH)
    target_link_libraries(sdso_ros
            sdso
            sdso_ros_setup
            ${Pangolin_LIBRARIES})
endif ()
