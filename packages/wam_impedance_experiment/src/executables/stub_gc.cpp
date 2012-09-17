#include <ros/ros.h>
#include <ros/package.h>

#include <cisstCommon/cmnConstants.h>

#include <cisstDevices/robotcomponents/controllers/devGravityCompensation.h>
#include "wam_impedance_experiment/devices/devStub.h"

#include <cisstMultiTask/mtsTaskManager.h>
#include <cisstVector/cisstVector.h>

#include <cisst_ros_integration/ros_tasks.h>


#include <sys/mman.h>

#include <boost/assign/std/vector.hpp>
#include <boost/assign/list_of.hpp>

#include <std_msgs/Empty.h>
#include <sensor_msgs/JointState.h>


void wam_joint_state_cisst_to_ros(
    const mtsVector<double> & joint_angles,
    sensor_msgs::JointState & msg)
{
  size_t n_joints = joint_angles.size();

  ROS_DEBUG_STREAM("Received "<<n_joints<<" joint angles...");
  using namespace boost::assign;

  msg.name +=
    "DarmSim/RightArm/WAM/YawJoint",
    "DarmSim/RightArm/WAM/ShoulderPitchJoint",
    "DarmSim/RightArm/WAM/ShoulderYawJoint",
    "DarmSim/RightArm/WAM/ElbowJoint",
    "DarmSim/RightArm/WAM/UpperWristYawJoint",
    "DarmSim/RightArm/WAM/UpperWristPitchJoint",
    "DarmSim/RightArm/WAM/LowerWristYawJoint";

  msg.position.resize(n_joints);
  for(size_t i=0; i<n_joints; i++) {
    msg.position[i] = joint_angles[i];
  }

  msg.velocity += 0,0,0,0,0,0,0;
  msg.effort += 0,0,0,0,0,0,0;
}

int main(int argc, char** argv){

  //mlockall(MCL_CURRENT | MCL_FUTURE);
  //RT_TASK task;
  //rt_task_shadow( &task, "main", 60, T_FPU );

  ros::init(argc, argv, "wam_stub_demo");

  //cmnLogger::SetMask(CMN_LOG_ALLOW_ALL);
  //cmnLogger::SetMaskDefaultLog(CMN_LOG_ALLOW_ALL);
  //cmnLogger::SetMaskFunction(CMN_LOG_ALLOW_ALL);
  //cmnLogger::SetMaskClassAll(CMN_LOG_ALLOW_ALL);


  mtsTaskManager* taskManager = mtsTaskManager::GetInstance();

  //devKeyboard kb;
  //kb.SetQuitKey( 'q' );
  //kb.AddKeyVoidFunction('G', "ctrlenable", devController::Enable);
  //taskManager->AddComponent( &kb );

  vctDynamicVector<double> qinit(7, 0.0);
  qinit[1] = -cmnPI_2;
  qinit[3] =  cmnPI;

  ROS_INFO("Creating stub...");

  devStub stub( "Stub", 0.02, OSA_CPU1, qinit );
  taskManager->AddComponent( &stub );

  ROS_INFO_STREAM("Getting robot description file...");

  std::string robfile("libs/etc/cisstRobot/WAM/wam7.rob");
  robfile = ros::package::getPath("cisst")+"/build/source/"+robfile;

  ROS_INFO_STREAM("Got robot description file: \""<<robfile<<"\"");

  vctMatrixRotation3<double> Rw0(
      0.0,  0.0, -1.0, 
      0.0,  1.0,  0.0, 
      1.0,  0.0,  0.0 );
  vctFixedSizeVector<double,3> tw0(0.0);
  vctFrame4x4<double> Rtw0( Rw0, tw0 );
  devGravityCompensation gc(
      "controller", 
      0.002,
      devController::DISABLED,
      OSA_CPU2,
      robfile,
      Rtw0 );
  taskManager->AddComponent( &gc );

  // Add ROS interface

  // Create an mtsRosPublisher that publishes data on the topic "cisst_data" at 2Hz
  const double publish_period = 100 * cmn_ms; // in milliseconds
  mtsRosTaskPeriodic * ros_periodic = new mtsRosTaskPeriodic("ROS_PERIODIC_ADAPTER",publish_period);

  FcnReadPublisherAdapter<sensor_msgs::JointState, mtsVector<double> > joint_state_adapter(
      "ReadRnPosition",
      "/joint_states",50,
      ros::NodeHandle(),
      &wam_joint_state_cisst_to_ros);

  ros_periodic->add_publisher(joint_state_adapter);
  taskManager->AddComponent( ros_periodic );



  mtsRosTaskFromSignal * ros_cb = new mtsRosTaskFromSignal("ROS_CB_ADAPTER");

  FcnVoidSubscriberAdapter<std_msgs::Empty> control_enable_adapter(
      devController::Enable,
      "/ctrl_enable",50);

  ros_cb->add_subscriber(control_enable_adapter);
  taskManager->AddComponent( ros_cb );


  // Connect everything
  //taskManager->Connect(
  //    kb.GetName(), "ctrlenable",
  //    gc.GetName(), devController::Control );
  taskManager->Connect(
      "ROS_CB_ADAPTER",  "RequiredAdapter",
      gc.GetName(), devController::Control );


  taskManager->Connect(
      gc.GetName(), devController::Output,
      stub.GetName(),devManipulator::Input );

  taskManager->Connect(
      gc.GetName(),   devController::Feedback,
			stub.GetName(),  devManipulator::Output );

  taskManager->Connect(
      "ROS_PERIODIC_ADAPTER",  "RequiredAdapter",
			stub.GetName(),  devManipulator::Output );

  taskManager->CreateAll();
  taskManager->StartAll();

  ros::spin();

  return 0;
}
