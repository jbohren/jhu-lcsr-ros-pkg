cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
rosbuild_genmsg()
#uncomment if you have defined services
rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

# FIXME: this is a hack (the headers at this path should be installed with cisst)
rosbuild_find_ros_package(cisst)
include_directories(${cisst_PACKAGE_PATH}/build/source/libs/include)

# build internal libraries
rosbuild_add_library(devices
  src/devices/devDAS6014.cpp
  src/devices/devKeyboardAAB.cpp
  src/devices/devStub.cpp)

rosbuild_add_library(controllers
  src/controllers/devMultipleControllers.cpp
  src/controllers/devImpedance.cpp
  src/controllers/devImpedanceIC.cpp
  src/controllers/devImpedanceTunable.cpp)

target_link_libraries(controllers gsl)

rosbuild_add_library(trajectories
  src/trajectories/devMultipleTrajectories.cpp
  src/trajectories/devForceInput.cpp
  src/trajectories/devReturnToZero.cpp
  src/trajectories/devStationary.cpp
  src/trajectories/devVelEqn.cpp)

rosbuild_add_library(robotcomponents
  src/robotcomponents/robManipulator_AAB.cpp
  src/robotcomponents/devWAM_AAB.cpp)

#build executables
#rosbuild_add_executable(experiment src/executables/experiment.cpp )
#target_link_libraries(experiment controllers trajectories devices)

#rosbuild_add_executable(wami_stub src/executables/wami_stub.cpp)
#target_link_libraries(wami_stub controllers trajectories devices comedi)

#rosbuild_add_executable(wami src/executables/wami.cpp)
#target_link_libraries(wami controllers trajectories devices comedi)

#rosbuild_add_executable(wami_fin2 src/executables/wami_fin2.cpp)
#target_link_libraries(wami_fin2 controllers trajectories devices comedi)

#rosbuild_add_executable(wami_fin src/executables/wami_fin.cpp)
#target_link_libraries(wami_fin controllers trajectories devices comedi)

#rosbuild_add_executable(wamiic src/executables/wamiic.cpp)
#target_link_libraries(wamiic controllers trajectories devices comedi)

#rosbuild_add_executable(wami_sine_home src/executables/wami_sine_home.cpp)
#target_link_libraries(wami_sine_home controllers trajectories devices comedi)

#rosbuild_add_executable(wami_sinewave src/executables/wami_sinewave.cpp)
#target_link_libraries(wami_sinewave controllers trajectories devices comedi)

#rosbuild_add_executable(stub_gc src/executables/stub_gc.cpp)
#target_link_libraries(stub_gc controllers trajectories devices comedi)

rosbuild_add_executable(daq src/executables/daq.cpp)
target_link_libraries(daq comedi)

rosbuild_add_executable(ctrlenabler src/executables/ctrlenabler.cpp)
target_link_libraries(ctrlenabler glut GL GLU robotcomponents)

rosbuild_add_executable(wam_gc src/executables/WAMgc.cpp)
target_link_libraries(wam_gc robotcomponents)

rosbuild_add_executable(wam_control src/executables/wam_control.cpp)
target_link_libraries(wam_control trajectories controllers devices comedi robotcomponents)

rosbuild_add_executable(two_wam_control src/executables/two_wam_control.cpp)
target_link_libraries(two_wam_control trajectories controllers devices comedi robotcomponents)

rosbuild_add_executable(stubs_control src/executables/stubs_control.cpp)
target_link_libraries(stubs_control trajectories controllers devices comedi robotcomponents)

rosbuild_add_executable(two_stubs_control src/executables/two_stubs_control.cpp)
target_link_libraries(two_stubs_control trajectories controllers devices comedi robotcomponents)

rosbuild_add_executable(logger src/executables/logger.cpp)
#target_link_libraries(logger trajectories controllers devices)

rosbuild_add_executable(graphics src/executables/graphics.cpp)
target_link_libraries(graphics glut GL GLU)

rosbuild_add_executable(experiment_manager src/executables/experiment_manager.cpp)
target_link_libraries(experiment_manager)

rosbuild_add_executable(projectiles src/executables/projectiles.cpp)
target_link_libraries(projectiles)
