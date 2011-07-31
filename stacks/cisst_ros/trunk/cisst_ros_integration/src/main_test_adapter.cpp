
/*****************************************************************************
 * main_test_adapter.cpp *
 * author: Jonathan Bohren 
 * created: July 30th, 2011
 *
 * This is a simple program for demonstrating ROS-CISST integration.
 *
 * It creates two CISST periodic tasks, one which increments a double (called
 * "SPEWER"), and one which publishes data over ROS (called "ROS_ADAPTER").
 * The ROS topic is specified below in this file, and is called "/cisst_data" by
 * default. The message type is std_msgs::Float64 (aka double).
 *
 * After starting a ROS master and this program, you can view the published
 * messages (a monotonically increasing number) by running:
 * $> rostopic echo /cisst_data
 ***************************************************************************/

// ROS includes
#include "cisst_ros_integration/mtsRosAdapter.h"
#include <std_msgs/Float64.h>

// CISST includes
#include <cisstCommon.h>
#include <cisstOSAbstraction.h>
#include <cisstMultiTask.h>

// App includes
#include "cisst_ros_integration/DataSpewer.h"


std_msgs::Float64 my_convert(const mtsDouble & data) {
  std_msgs::Float64 msg;

  msg.data = data.GetData();

  return msg;
}


int main(int argc, char** argv)
{
  // Initialize ROS node
  ros::init(argc, argv, "cisst_publisher");
  ros::NodeHandle nh;

  // Configure CISST logging
  cmnLogger::SetMask(CMN_LOG_ALLOW_ALL);

  // Create a DataSpewer to generate data to publish over ROS
  const double spew_period = 100 * cmn_ms; // in milliseconds
  DataSpewer * spewer_tsk = new DataSpewer("SPEWER", spew_period, "cisst_data");

  // Create an mtsRosPublisher that publishes data on the topic "cisst_data" at 2Hz
  const double publish_period = 500 * cmn_ms; // in milliseconds
  mtsRosAdapter * adapter_tsk = new mtsRosAdapter("ROS_ADAPTER",publish_period);

  // You can use the predefined conversion functions in cisst_ros_integration/conversions.h
  adapter_tsk->add_publisher<std_msgs::Float64, mtsDouble>(
      "/cisst_data",50,
      "cisst_data");

  adapter_tsk->add_subscriber<std_msgs::Float64, mtsDouble>(
      "/cisst_data",50,
      "cisst_data");

  /** Or you can give it a conversion function explicitly
  adapter_tsk->add_publisher<std_msgs::Float64, mtsDouble>(
      nh.advertise<std_msgs::Float64>("/cisst_data",50),
      "cisst_data",
      &my_convert);
      **/

  // Create CISST task manager
  mtsTaskManager * taskManager = mtsTaskManager::GetInstance();

  // Add the tasks to the task manager
  taskManager->AddComponent(spewer_tsk);
  taskManager->AddComponent(adapter_tsk);
  //
  // Connect the tasks, task.RequiresInterface -> task.ProvidesInterface
  taskManager->Connect("SPEWER", "PublishInterface", "ROS_ADAPTER", "PublishInterface");

  // Create the task connections
  taskManager->CreateAll();
  // Start the CISST tasks
  taskManager->StartAll();

  // Wait for user to kill the program
  ros::spin();

  // Cleanup, and wait for taks to finish
  taskManager->KillAll();

  osaSleep(publish_period * 2);
  while (!adapter_tsk->IsTerminated()) { osaSleep(publish_period); }
  while (!spewer_tsk->IsTerminated()) { osaSleep(spew_period); }

  taskManager->Cleanup();

  return 0;
}
