#ifndef __CISST_ROS_INTEGRATION__DATA_SPEWER__
#define __CISST_ROS_INTEGRATION__DATA_SPEWER__

/*****************************************************************************
 * mtsRosPublisher *
 * author: Jonathan Bohren 
 * created: June 10th, 2011
 *
 * This is a simple CISST mtsTaskPeriodic component which "spews" data over a
 * required interface called "PublishInterface" by calling the function proxy
 * "push_data".
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
  double data_;
  // CISST proxy to ROS publish function
  mtsFunctionWrite push_data_;

public:

  DataSpewer(
      const std::string & task_name,
      double period,
      const std::string & push_cmd,
      const std::string & pull_cmd) :
    mtsTaskPeriodic(task_name, period, false, 5000),
    data_(0)
  {
    // Announce creation of publisher
    ROS_INFO_STREAM("Started data spewer.");
    // Create an interface to local methods
    mtsInterfaceRequired * push_interface = this->AddInterfaceRequired("PushInterface");
    if( !push_interface ) {
      ROS_ERROR_STREAM("Could not create MTS interface of CISST-ROS publisher.");
      exit(-1);
    }
    
    // Add the publish function to the command interface
    if( !push_interface->AddFunction(push_cmd.c_str(), this->push_data_) ) {
      ROS_ERROR_STREAM("Could not add push command to MTS interface of CISST-ROS publisher.");
      exit(-1);
    }

    mtsInterfaceProvided * pull_interface = this->AddInterfaceProvided("PullInterface");

    if( !pull_interface->AddCommandRead(&DataSpewer::pull_double, this, pull_cmd.c_str()) ) {
      ROS_ERROR_STREAM("Could not add pull command to MTS interface of CISST-ROS publisher.");
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
    data_++;
    // Publish data via CISST interface to actual ROS publisher
    push_data_(data_);
  }

  void pull_double(mtsDouble & data) const {
    data = data_;
  }

  void Cleanup(void) {};
};

CMN_DECLARE_SERVICES_INSTANTIATION(DataSpewer);


#endif // ifndef __CISST_ROS_INTEGRATION__DATA_SPEWER__
