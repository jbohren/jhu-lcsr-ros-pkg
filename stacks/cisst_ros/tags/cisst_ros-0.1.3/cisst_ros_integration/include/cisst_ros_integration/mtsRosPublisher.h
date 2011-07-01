#ifndef __CISST_ROS_INTEGRATION__MTS_ROS_PUBLISHER__
#define __CISST_ROS_INTEGRATION__MTS_ROS_PUBLISHER__

/*****************************************************************************
 * mtsRosPublisher *
 * author: Jonathan Bohren 
 * created: June 10th, 2011
 *
 * This is a simple example of a CISST mtsTaskPeriodic which also publishes
 * data out over ROS. The specific ROS topic that it advertises is given at
 * construction.
 *
 * This task provides a CISST interface to its internal ros::Publisher. The
 * interface is called "PublishInterface" and it provides a single command
 * called "publish_double".
 *
 * Note that at this point, it can only send out messages of type
 * std_msgs::Float64 (aka double). It is, however, trivial to extend this
 * capability to other datatypes.
 ***************************************************************************/

// ROS includes
#include <ros/ros.h>
#include <std_msgs/Float64.h>

// ROS realtime publisher is not necessarily needed, since this task
// can have its own thread.
//#include <realtime_tools/realtime_publisher.h>

// CISST includes
#include <cisstMultiTask.h>

//TODO: template <typename msg_T>
class mtsRosPublisher : public mtsTaskPeriodic {
  // CISST log configuration "Run Error" by default
  CMN_DECLARE_SERVICES(CMN_NO_DYNAMIC_CREATION, CMN_LOG_LOD_RUN_ERROR);

protected:
  // ROS NodeHandle
  ros::NodeHandle nh_;
  // Actual ROS publisher
  ros::Publisher pub_;

  // Publish function
  void publish_double(const mtsDouble & val) {
    ROS_DEBUG_STREAM("CISST-ROS publisher publishing value: "<<(double)val<<"...");
    // Create message
    std_msgs::Float64 msg;
    msg.data = (double)val;
    // Publish value
    pub_.publish(msg);
  }

public:

  mtsRosPublisher(
      const std::string & task_name,
      double period,
      const std::string & topic_name) :
    mtsTaskPeriodic(task_name, period, false, 5000),
    nh_(),
    pub_(nh_.advertise<std_msgs::Float64>(topic_name,1000))
  {
    // Announce creation of publisher
    ROS_INFO_STREAM("Started CISST-ROS publisher on topic \""<<topic_name<<"\"");

    // Create an interface to local methods
    mtsInterfaceProvided * publish_interface = this->AddInterfaceProvided("PublishInterface");
    if( !publish_interface ) {
      ROS_ERROR_STREAM("Could not create MTS interface of CISST-ROS publisher.");
      exit(-1);
    }

    // Add the publisch function to the command interface
    if( !(publish_interface->AddCommandWrite( &mtsRosPublisher::publish_double, this, "publish_double"))) {
      ROS_ERROR_STREAM("Could not add command to MTS interface of CISST-ROS publisher.");
      exit(-1);
    }
  }

  ~mtsRosPublisher() {};

  void Configure(const std::string & CMN_UNUSED(filename)) {};
  void Startup(void) {};

  void Run(void) {
    // Process the CISST commands/events
    this->ProcessQueuedCommands();
    this->ProcessQueuedEvents();
  }

  void Cleanup(void) {};
};

CMN_DECLARE_SERVICES_INSTANTIATION(mtsRosPublisher);

#endif // ifndef __CISST_ROS_INTEGRATION__MTS_ROS_PUBLISHER__
