
// ROS includes
#include <std_msgs/Float64.h>

// CISST includes
#include <cisstCommon.h>
#include <cisstOSAbstraction.h>
#include <cisstMultiTask.h>

// App includes
#include <cisst_ros_integration/DataSpewer.h>
#include <cisst_ros_integration/ros_tasks.h>


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
  mtsRosTaskFromSignal * adapter_tsk = new mtsRosTaskFromSignal("ROS_ADAPTER");

  // Create an adapter to publish to "/cisst_data"
  WriteRosPublisherAdapter<std_msgs::Float64, mtsDouble> write_adapter(
      "cisst_data",
      "/cisst_data",50);

  adapter_tsk->add_publisher(write_adapter);

  // Create CISST task manager
  mtsTaskManager * taskManager = mtsTaskManager::GetInstance();

  // Add the tasks to the task manager
  taskManager->AddComponent(spewer_tsk);
  taskManager->AddComponent(adapter_tsk);
  //
  // Connect the tasks, task.RequiresInterface -> task.ProvidesInterface
  taskManager->Connect("SPEWER", "PublishInterface", "ROS_ADAPTER", "ProvidedInterface");

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
