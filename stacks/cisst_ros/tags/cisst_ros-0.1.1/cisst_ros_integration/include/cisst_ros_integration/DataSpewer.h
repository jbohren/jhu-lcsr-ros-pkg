#ifndef __CISST_ROS_INTEGRATION__DATA_SPEWER__
#define __CISST_ROS_INTEGRATION__DATA_SPEWER__

/*****************************************************************************
 * mtsRosPublisher *
 * author: Jonathan Bohren 
 * created: June 10th, 2011
 *
 * This is a simple CISST mtsTaskPeriodic component which "spews" data over a
 * required interface called "PublishInterface" by calling the function proxy
 * "publish_double".
 *
 * This is simply a command request generator for testing out the ROS-CISST
 * integration.
 ***************************************************************************/

// ROS includes
#include <ros/ros.h>

// CISST includes
#include <cisstMultiTask.h>

class DataSpewer : public mtsTaskPeriodic {
  // CISST log configuration "Run Error" by default
  CMN_DECLARE_SERVICES(CMN_NO_DYNAMIC_CREATION, CMN_LOG_LOD_RUN_ERROR);

protected:
  // Data storage
  double data;
  // CISST proxy to ROS publish function
  mtsFunctionWrite publish_double;

public:

  DataSpewer(
      const std::string & task_name,
      double period) :
    mtsTaskPeriodic(task_name, period, false, 5000),
    data(0)
  {
    // Announce creation of publisher
    ROS_INFO_STREAM("Started data spewer.");
    // Create an interface to local methods
    mtsInterfaceRequired * publish_interface = this->AddInterfaceRequired("PublishInterface");
    if( !publish_interface ) {
      ROS_ERROR_STREAM("Could not create MTS interface of CISST-ROS publisher.");
      exit(-1);
    }

    // Add the publish function to the command interface
    if( !publish_interface->AddFunction("publish_double", this->publish_double) ) {
      ROS_ERROR_STREAM("Could not add command to MTS interface of CISST-ROS publisher.");
      exit(-1);
    }
  }

  ~DataSpewer() {};

  void Configure(const std::string & CMN_UNUSED(filename)) {};
  void Startup(void) {};

  void Run(void) {
    // Process the CISST commands/events
    this->ProcessQueuedCommands() ;
    this->ProcessQueuedEvents() ;

    // Increment data
    data++;
    // Publish data via CISST interface to actual ROS publisher
    publish_double(data);
  }

  void Cleanup(void) {};
};

CMN_DECLARE_SERVICES_INSTANTIATION(DataSpewer);


#endif // ifndef __CISST_ROS_INTEGRATION__DATA_SPEWER__
