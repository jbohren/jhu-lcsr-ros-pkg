#ifndef __CISST_ROS_INTEGRATION__MTS_ROS_ADAPTER__
#define __CISST_ROS_INTEGRATION__MTS_ROS_ADAPTER__

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

#include <cisst_ros_integration/conversions.h>
// ROS realtime publisher is not necessarily needed, since this task
// can have its own thread.
//#include <realtime_tools/realtime_publisher.h>

// CISST includes
#include <cisstMultiTask.h>

class mtsRosAdapter : public mtsTaskPeriodic {
  // CISST log configuration "Run Error" by default
  CMN_DECLARE_SERVICES(CMN_NO_DYNAMIC_CREATION, CMN_LOG_LOD_RUN_ERROR);

protected:

  class AdapterBase {};

  template<typename MSG_T, typename CISST_T>
    class PublishAdapter : public AdapterBase {
    private:
      ros::Publisher pub_;
      boost::function<MSG_T(const CISST_T &)> convert_;

    public:
      PublishAdapter(ros::Publisher pub, boost::function<MSG_T(const CISST_T &)> convert ) :
        pub_(pub),
        convert_(convert)
      {
      }

      void publish(const CISST_T & data) {
        ROS_DEBUG_STREAM("CISST-ROS publisher publishing value: "<<data<<"...");
        // Publish value
        pub_.publish(convert_(data));
      }
    };

  std::vector<AdapterBase*> pub_adapters_;

  mtsInterfaceProvided * publish_interface_;
public:

  mtsRosAdapter(
      const std::string & task_name,
      double period) :
    mtsTaskPeriodic(task_name, period, false, 5000)
  {
    // Create CISST interfaces to ROS interfaces
    publish_interface_ = this->AddInterfaceProvided("PublishInterface");
    if( !publish_interface_ ) {
      ROS_ERROR_STREAM("Could not create MTS interface for CISST-ROS publisher.");
      exit(-1);
    }

  }

  ~mtsRosAdapter() {
    for(size_t i=0; i<pub_adapters_.size(); i++) {
      delete pub_adapters_[i];
    }
  
  };

  template<typename MSG_T, typename CISST_T>
  bool add_publisher(
      ros::Publisher pub,
      std::string cmd_name,
      MSG_T (*convert)(const CISST_T &) = &cisst_ros_integration::convert<MSG_T,CISST_T>) {
    // Announce creation of publisher
    ROS_INFO_STREAM("Connecting CISST-ROS publisher on topic \""<<pub.getTopic()<<"\"");

    // Create workaround class to deal with CISST command API restrictions
    PublishAdapter<MSG_T, CISST_T> *wnd = new PublishAdapter<MSG_T, CISST_T>(pub, convert);
    pub_adapters_.push_back(wnd);

    // Add the publish function to the command interface
    mtsCommandWriteBase* cmd = publish_interface_->AddCommandWrite(
        &PublishAdapter<MSG_T, CISST_T>::publish,
        wnd,
        cmd_name);

    if( !cmd ) {
      ROS_ERROR_STREAM("Could not add command to MTS interface of CISST-ROS publisher.");
      return false;
    }

    return true;
  }

  ////

  void Configure(const std::string & CMN_UNUSED(filename)) {};
  void Startup(void) {};

  void Run(void) {
    // Process the CISST commands/events
    this->ProcessQueuedCommands();
    this->ProcessQueuedEvents();
  }

  void Cleanup(void) {};
};

CMN_DECLARE_SERVICES_INSTANTIATION(mtsRosAdapter);

#endif // ifndef __CISST_ROS_INTEGRATION__MTS_ROS_ADAPTER__
