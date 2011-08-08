#ifndef __CISST_ROS_INTEGRATION__ROS_TASK_H__
#define __CISST_ROS_INTEGRATION__ROS_TASK_H__

#include <stdexcept>

// ROS includes
#include <ros/ros.h>
#include <std_msgs/Float64.h>

#include <cisst_ros_integration/conversions.h>

// CISST includes
#include <cisstMultiTask.h>

class RosPublisherAdapter
{
public:
  virtual void init(mtsInterfaceRequired *required_interface, mtsInterfaceProvided *provided_interface) = 0;
  virtual void publish() { };
};

template <typename ROS_T, typename CISST_RES_T>
class VoidReturnRosPublisherAdapter : public RosPublisherAdapter
{
public:
  VoidReturnRosPublisherAdapter(
      std::string cisst_cmd_name,
      std::string ros_topic_name,
      uint32_t ros_queue_size,
      ros::NodeHandle ros_node_handle = ros::NodeHandle(),
      boost::function<void(const CISST_RES_T &, ROS_T &)> convert_fun = &cisst_ros_integration::cisst_to_ros<ROS_T,CISST_RES_T>) : 
    pub_(ros_node_handle.advertise<ROS_T>(ros_topic_name, ros_queue_size)),
    cisst_cmd_name_(cisst_cmd_name),
    ros_topic_name_(ros_topic_name),
    cisst_function_(),
    convert_fun_(convert_fun)
  { }

  virtual void init(mtsInterfaceRequired *required_interface, mtsInterfaceProvided *provided_interface)
  {
    bool added = required_interface->AddFunction(cisst_cmd_name_, cisst_function_, MTS_OPTIONAL);
    if( !added ) {
      ROS_ERROR_STREAM("Could not add function to MTS interface of CISST-ROS publisher.");
      throw std::runtime_error("Could not add function to MTS interface.");
    }
    
    // Announce creation of publisher
    ROS_INFO_STREAM("Connected CISST command \""<<cisst_cmd_name_<<"\" to ROS publisher on topic \""<<ros_topic_name_<<"\"");
  }

  void publish()
  {
    // Get data from CISST interface
    CISST_RES_T data;
    cisst_function_(data);

    // Convert CISST data to ROS message
    ROS_T msg;
    convert_fun_(data, msg);

    // Convert data from and publish it
    pub_.publish(msg);
  }

private:
  ros::Publisher pub_;
  std::string cisst_cmd_name_;
  std::string ros_topic_name_;
  mtsFunctionVoidReturn cisst_function_;
  boost::function<void(const CISST_RES_T &, ROS_T &)> convert_fun_;
};

template <typename ROS_T, typename CISST_RES_T>
class WriteRosPublisherAdapter : public RosPublisherAdapter
{
public:
  WriteRosPublisherAdapter(
      std::string cisst_cmd_name,
      std::string ros_topic_name,
      uint32_t ros_queue_size,
      ros::NodeHandle ros_node_handle = ros::NodeHandle(),
      boost::function<void(const CISST_RES_T &, ROS_T &)> convert_fun = &cisst_ros_integration::cisst_to_ros<ROS_T,CISST_RES_T>) : 
    pub_(ros_node_handle.advertise<ROS_T>(ros_topic_name, ros_queue_size)),
    cisst_cmd_name_(cisst_cmd_name),
    ros_topic_name_(ros_topic_name),
    cisst_cmd_(NULL),
    convert_fun_(convert_fun)
  { }

  void init(mtsInterfaceRequired *required_interface, mtsInterfaceProvided *provided_interface)
  {
    // Add the publish function to the command interface
    cisst_cmd_ = provided_interface->AddCommandWrite(
        &WriteRosPublisherAdapter<ROS_T, CISST_RES_T>::publish,
        this,
        cisst_cmd_name_);

    if( !cisst_cmd_ ) {
      ROS_ERROR_STREAM("Could not add function to MTS interface of CISST-ROS publisher.");
      throw std::runtime_error("Could not add function to MTS interface.");
    }
    
    // Announce creation of publisher
    ROS_INFO_STREAM("Connected CISST command \""<<cisst_cmd_name_<<"\" to ROS publisher on topic \""<<ros_topic_name_<<"\"");
  }

  void publish(const CISST_RES_T &data)
  {
    // Convert CISST data to ROS message
    ROS_T msg;
    convert_fun_(data, msg);

    // Convert data from and publish it
    pub_.publish(msg);
  }

private:
  ros::Publisher pub_;
  std::string cisst_cmd_name_;
  std::string ros_topic_name_;
  mtsCommandWriteBase *cisst_cmd_;
  boost::function<void(const CISST_RES_T &, ROS_T &)> convert_fun_;
};

///////////////////////////////////////////////////////////////////////////////

class mtsRosTaskPeriodic : public mtsTaskPeriodic
{
  // CISST log configuration "Run Error" by default
  CMN_DECLARE_SERVICES(CMN_NO_DYNAMIC_CREATION, CMN_LOG_LOD_RUN_ERROR);

public:
  mtsRosTaskPeriodic(const std::string & task_name, double period) :
    mtsTaskPeriodic( task_name, period, false, 500)
  {
    // Construct CISST interface
    required_interface_ = this->AddInterfaceRequired("RequiredAdapter");
    if( !required_interface_ ) {
      ROS_ERROR_STREAM("Could not create MTS interface for CISST-ROS publisher.");
      throw std::runtime_error("Could not add function to MTS interface.");
    }
  }

  ~mtsRosTaskPeriodic() { };

  void Configure(const std::string & CMN_UNUSED(filename)) {};
  void Startup(void) {};
  void Cleanup(void) {};

  void Run(void)
  {
    // Process the CISST commands/events
    this->ProcessQueuedCommands();
    this->ProcessQueuedEvents();
    
    // Publish data
    for(size_t i=0; i<publishers_.size(); i++) {
      publishers_[i]->publish();
    }
  }

  void add_publisher(RosPublisherAdapter & pub_adapter)
  {
    pub_adapter.init(required_interface_, NULL);
  }

  std::vector<RosPublisherAdapter*> publishers_;
  mtsInterfaceRequired *required_interface_;

};

CMN_DECLARE_SERVICES_INSTANTIATION(mtsRosTaskPeriodic);

class mtsRosTaskFromSignal : public mtsTaskFromSignal
{
  // CISST log configuration "Run Error" by default
  CMN_DECLARE_SERVICES(CMN_NO_DYNAMIC_CREATION, CMN_LOG_LOD_RUN_ERROR);

public:
  mtsRosTaskFromSignal(const std::string & task_name) :
    mtsTaskFromSignal( task_name, 500)
  {
    // Construct CISST interface
    provided_interface_ = this->AddInterfaceProvided("ProvidedAdapter");
    if( !provided_interface_ ) {
      ROS_ERROR_STREAM("Could not create MTS interface for CISST-ROS publisher.");
      throw std::runtime_error("Could not add function to MTS interface.");
    }

    required_interface_ = this->AddInterfaceRequired("RequiredAdapter");
    if( !required_interface_ ) {
      ROS_ERROR_STREAM("Could not create MTS interface for CISST-ROS publisher.");
      throw std::runtime_error("Could not add function to MTS interface.");
    }
  }

  ~mtsRosTaskFromSignal() { };

  void Configure(const std::string & CMN_UNUSED(filename)) {};
  void Startup(void) {};
  void Cleanup(void) {};

  void Run(void)
  {
    // Process the CISST commands/events
    this->ProcessQueuedCommands();
    this->ProcessQueuedEvents();
    
    // Publish data 
    for(size_t i=0; i<publishers_.size(); i++) {
      publishers_[i]->publish();
    }
  }

  void add_publisher(RosPublisherAdapter & pub_adapter)
  {
    pub_adapter.init(required_interface_, provided_interface_);
  }

  mtsInterfaceRequired *required_interface_;
  mtsInterfaceProvided *provided_interface_;

  std::vector<RosPublisherAdapter*> publishers_;
  //std::vector<RosSubscriberAdapter*> subscribers_;

};

CMN_DECLARE_SERVICES_INSTANTIATION(mtsRosTaskFromSignal);

#endif // ifndef __CISST_ROS_INTEGRATION__ROS_TASK_H__
