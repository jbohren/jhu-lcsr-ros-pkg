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

  // Local classes for adapting CISST and ROS interfaces
  class AdapterBase {};

  template<typename ROS_T, typename MTS_T>
    class PublishAdapter : public AdapterBase
  {
  private:
    ros::Publisher pub_;
    boost::function<ROS_T(const MTS_T &)> convert_fun_;

  public:
    PublishAdapter(ros::Publisher pub, boost::function<ROS_T(const MTS_T &)> convert_fun ) :
      pub_(pub),
      convert_fun_(convert_fun)
    {
    }

    void publish(const MTS_T & data)
    {
      ROS_DEBUG_STREAM("CISST-ROS publisher publishing value: "<<data<<"...");
      // Publish value
      pub_.publish(convert_fun_(data));
    }
  };

  template<typename ROS_T, typename MTS_T>
    class SubscribeAdapter : public AdapterBase
  {
  private:
    ros::Subscriber sub_;
    boost::function<MTS_T(const ROS_T &)> convert_fun_;
    mtsFunctionWrite mts_fun_;

  public:
    SubscribeAdapter(
        boost::function<MTS_T(const ROS_T &)> convert_fun,
        mtsFunctionWrite mts_fun) :
      sub_(),
      convert_fun_(convert_fun),
      mts_fun_(mts_fun)
    {
      
    }

    // Wrapper to act as the callback for the subscriber
    void receive(const ROS_T & msg)
    {
      // Call CISST function
      mts_fun_(convert_fun_(msg));
    }

    void set_subscriber(ros::Subscriber sub)
    {
      sub_ = sub;
    }
  };

public:

  mtsRosAdapter(
      const std::string & task_name,
      double period) :
    mtsTaskPeriodic(task_name, period, false, 5000),
    nh_(),
    adapters_(0),
    publish_interface_(NULL),
    subscribe_interface_(NULL)
  {
    // Create CISST interface for publishers
    // This interface accepts commands from CISST components and passes them on to ROS publishers
    publish_interface_ = this->AddInterfaceProvided("PublishInterface");
    if( !publish_interface_ ) {
      ROS_ERROR_STREAM("Could not create MTS interface for CISST-ROS publisher.");
      exit(-1);
    }

    // Create CISST interface for subscribers
    // This interface is activated by callbacks from ROS subscribers and sends commands to CISST components
    subscribe_interface_ = this->AddInterfaceRequired("SubscribeInterface", MTS_OPTIONAL);
    if( !publish_interface_ ) {
      ROS_ERROR_STREAM("Could not create MTS interface of CISST-ROS subscriber.");
      exit(-1);
    }
  }

  ~mtsRosAdapter()
  {
    // Delete adapters
    for(size_t i=0; i<adapters_.size(); i++) {
      delete adapters_[i];
    }
  };

  // Add ROS publisher to this task
  template<typename ROS_T, typename MTS_T>
  bool add_publisher(
      std::string topic_name,
      uint32_t queue_size,
      std::string cmd_name,
      ROS_T (*convert_fun)(const MTS_T &) = &cisst_ros_integration::ros_from_mts<ROS_T,MTS_T>)
  {
    // Announce creation of publisher
    ROS_INFO_STREAM("Connecting CISST-ROS publisher on topic \""<<topic_name<<"\"");

    // Create publisher with this instance's node handle
    ros::Publisher pub = nh_.advertise<ROS_T>(topic_name, queue_size);

    // Create workaround class to deal with CISST command API restrictions
    PublishAdapter<ROS_T, MTS_T> *wnd = new PublishAdapter<ROS_T, MTS_T>(pub, convert_fun);
    adapters_.push_back(wnd);

    // Add the publish function to the command interface
    mtsCommandWriteBase* cmd = publish_interface_->AddCommandWrite(
        &PublishAdapter<ROS_T, MTS_T>::publish,
        wnd,
        cmd_name);

    if( !cmd ) {
      ROS_ERROR_STREAM("Could not add command to MTS interface of CISST-ROS publisher.");
      return false;
    }

    return true;
  }

  // Add ROS subscriber to this task
  template<typename ROS_T, typename MTS_T>
  bool add_subscriber(
      std::string topic_name,
      uint32_t queue_size,
      std::string cmd_name,
      MTS_T (*convert_fun)(const ROS_T &) = &cisst_ros_integration::mts_from_ros<ROS_T,MTS_T>)
  {
    // Announce creation of publisher
    ROS_INFO_STREAM("Connecting CISST-ROS subscriber on topic \""<<topic_name<<"\"");

    // Create function proxy
    mtsFunctionWrite *mts_fun = new mtsFunctionWrite();
    // Add the function proxy to the CISST interface
    subscribe_interface_->AddFunction(cmd_name, *mts_fun);

    // Create an adapter to 
    SubscribeAdapter<ROS_T, MTS_T> *sub_adapter = new SubscribeAdapter<ROS_T, MTS_T>(convert_fun, *mts_fun);
    adapters_.push_back(sub_adapter);

    // Create subscriber with instance node handle
    ros::Subscriber sub = nh_.subscribe(topic_name, queue_size, &SubscribeAdapter<ROS_T, MTS_T>::receive, sub_adapter);

    // Save the subscriber handle
    sub_adapter->set_subscriber(sub);
    
    return true;
  }

  void Configure(const std::string & CMN_UNUSED(filename)) {};
  void Startup(void) {};

  void Run(void) {
    // Process the CISST commands/events
    this->ProcessQueuedCommands();
    this->ProcessQueuedEvents();
  }

  void Cleanup(void) {};

private:
  ros::NodeHandle nh_;
  std::vector<AdapterBase*> adapters_;
  mtsInterfaceProvided * publish_interface_;
  mtsInterfaceRequired * subscribe_interface_;
};

CMN_DECLARE_SERVICES_INSTANTIATION(mtsRosAdapter);

#endif // ifndef __CISST_ROS_INTEGRATION__MTS_ROS_ADAPTER__
