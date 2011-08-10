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

template <typename ROS_T>
class FcnVoidPublisherAdapter : public RosPublisherAdapter
{
public:
  FcnVoidPublisherAdapter(
      std::string cisst_cmd_name,
      std::string ros_topic_name,
      uint32_t ros_queue_size,
      ROS_T default_msg = ROS_T(),
      ros::NodeHandle ros_node_handle = ros::NodeHandle()) : 
    pub_(ros_node_handle.advertise<ROS_T>(ros_topic_name, ros_queue_size)),
    default_msg_(default_msg),
    cisst_cmd_name_(cisst_cmd_name),
    ros_topic_name_(ros_topic_name),
    cisst_function_()
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
    ROS_DEBUG_STREAM("Publishing by calling mtsFunctionVoid \""<<cisst_cmd_name_<<"\"");

    // Get data from CISST interface
    cisst_function_();

    // Convert data from and publish it
    pub_.publish(default_msg_);
  }

private:
  ros::Publisher pub_;
  ROS_T default_msg_;
  std::string cisst_cmd_name_;
  std::string ros_topic_name_;
  mtsFunctionVoid cisst_function_;
};

template <typename ROS_T, typename CISST_RES_T>
class FcnVoidReturnPublisherAdapter : public RosPublisherAdapter
{
public:
  FcnVoidReturnPublisherAdapter(
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
    ROS_DEBUG_STREAM("Publishing by calling mtsFunctionVoidReturn \""<<cisst_cmd_name_<<"\"");

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
class FcnReadPublisherAdapter : public RosPublisherAdapter
{
public:
  FcnReadPublisherAdapter(
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

    ROS_DEBUG_STREAM("Publishing by calling mtsFunctionRead \""<<cisst_cmd_name_<<"\"; "<<data<<" --> "<<msg);
  }

private:
  ros::Publisher pub_;
  std::string cisst_cmd_name_;
  std::string ros_topic_name_;
  mtsFunctionRead cisst_function_;
  boost::function<void(const CISST_RES_T &, ROS_T &)> convert_fun_;
};

template <typename ROS_T>
class CmdVoidPublisherAdapter : public RosPublisherAdapter
{
public:
  CmdVoidPublisherAdapter(
      std::string cisst_cmd_name,
      std::string ros_topic_name,
      uint32_t ros_queue_size,
      ROS_T default_msg = ROS_T(),
      ros::NodeHandle ros_node_handle = ros::NodeHandle()) :
    pub_(ros_node_handle.advertise<ROS_T>(ros_topic_name, ros_queue_size)),
    default_msg_(default_msg),
    cisst_cmd_name_(cisst_cmd_name),
    ros_topic_name_(ros_topic_name),
    cisst_cmd_(NULL)
  { }

  void init(mtsInterfaceRequired *required_interface, mtsInterfaceProvided *provided_interface)
  {
    // Add the publish function to the command interface
    cisst_cmd_ = provided_interface->AddCommandVoid(
        &CmdVoidPublisherAdapter<ROS_T>::publish,
        this,
        cisst_cmd_name_);

    if( !cisst_cmd_ ) {
      ROS_ERROR_STREAM("Could not add function to MTS interface of CISST-ROS publisher.");
      throw std::runtime_error("Could not add function to MTS interface.");
    }
    
    // Announce creation of publisher
    ROS_INFO_STREAM("Connected CISST command \""<<cisst_cmd_name_<<"\" to ROS publisher on topic \""<<ros_topic_name_<<"\"");
  }

  void publish()
  {
    ROS_DEBUG_STREAM("Publishing by receiving call to mtsFunctionWrite \""<<cisst_cmd_name_<<"\"");

    // Convert data from and publish it
    pub_.publish(default_msg_);
  }

private:
  ros::Publisher pub_;
  ROS_T default_msg_;
  std::string cisst_cmd_name_;
  std::string ros_topic_name_;
  mtsCommandWriteBase *cisst_cmd_;
};

template <typename ROS_T, typename CISST_RES_T>
class CmdWritePublisherAdapter : public RosPublisherAdapter
{
public:
  CmdWritePublisherAdapter(
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
        &CmdWritePublisherAdapter<ROS_T, CISST_RES_T>::publish,
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
    ROS_DEBUG_STREAM("Publishing by receiving call to mtsFunctionWrite \""<<cisst_cmd_name_<<"\"");

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

class RosSubscriberAdapter
{
public:
  virtual void init(mtsInterfaceRequired *required_interface, mtsInterfaceProvided *provided_interface) = 0;
};

template <typename ROS_T>
class FcnVoidSubscriberAdapter : public RosSubscriberAdapter
{
public:
  FcnVoidSubscriberAdapter(
      std::string cisst_cmd_name,
      std::string ros_topic_name,
      uint32_t ros_queue_size,
      ros::NodeHandle ros_node_handle = ros::NodeHandle()) :
    sub_(ros_node_handle.subscribe<ROS_T>(
          ros_topic_name, ros_queue_size,
          &FcnVoidSubscriberAdapter::receive, this)),
    cisst_cmd_name_(cisst_cmd_name),
    ros_topic_name_(ros_topic_name),
    cisst_function_()
  { }

  void init(mtsInterfaceRequired *required_interface, mtsInterfaceProvided *provided_interface)
  {
    bool added = required_interface->AddFunction(cisst_cmd_name_, cisst_function_, MTS_OPTIONAL);
    if( !added ) {
      ROS_ERROR_STREAM("Could not add function to MTS interface of CISST-ROS publisher.");
      throw std::runtime_error("Could not add function to MTS interface.");
    }
    
    // Announce creation of publisher
    ROS_INFO_STREAM("Connected CISST command \""<<cisst_cmd_name_<<"\" to ROS publisher on topic \""<<ros_topic_name_<<"\"");
  }

  void receive(const ROS_T & msg)
  {
    // Convert data and call CISST function
    cisst_function_();
  }

private:
  ros::Subscriber sub_;
  std::string cisst_cmd_name_;
  std::string ros_topic_name_;
  mtsFunctionWrite cisst_function_;
};

template <typename ROS_T, typename CISST_RES_T>
class FcnWriteSubscriberAdapter : public RosSubscriberAdapter
{
public:
  FcnWriteSubscriberAdapter(
      std::string cisst_cmd_name,
      std::string ros_topic_name,
      uint32_t ros_queue_size,
      ros::NodeHandle ros_node_handle = ros::NodeHandle(),
      boost::function<void(const ROS_T &, CISST_RES_T &)> convert_fun = &cisst_ros_integration::ros_to_cisst<ROS_T,CISST_RES_T>) : 
    sub_(ros_node_handle.subscribe<ROS_T>(
          ros_topic_name, ros_queue_size,
          &FcnWriteSubscriberAdapter::receive, this)),
    cisst_cmd_name_(cisst_cmd_name),
    ros_topic_name_(ros_topic_name),
    cisst_function_(),
    convert_fun_(convert_fun)
  { }

  void init(mtsInterfaceRequired *required_interface, mtsInterfaceProvided *provided_interface)
  {
    bool added = required_interface->AddFunction(cisst_cmd_name_, cisst_function_, MTS_OPTIONAL);
    if( !added ) {
      ROS_ERROR_STREAM("Could not add function to MTS interface of CISST-ROS publisher.");
      throw std::runtime_error("Could not add function to MTS interface.");
    }
    
    // Announce creation of publisher
    ROS_INFO_STREAM("Connected CISST command \""<<cisst_cmd_name_<<"\" to ROS publisher on topic \""<<ros_topic_name_<<"\"");
  }

  void receive(const ROS_T & msg)
  {
    // Convert data and call CISST function
    CISST_RES_T data;
    convert_fun_(msg, data);

    cisst_function_(data);
  }

private:
  ros::Subscriber sub_;
  std::string cisst_cmd_name_;
  std::string ros_topic_name_;
  mtsFunctionWrite cisst_function_;
  boost::function<void(const ROS_T &, CISST_RES_T &)> convert_fun_;
};

///////////////////////////////////////////////////////////////////////////////

class mtsRosTask
{
public:
  mtsRosTask(mtsTask* task) {
    // Construct CISST interface
    provided_interface_ = task->AddInterfaceProvided("ProvidedAdapter");
    if( !provided_interface_ ) {
      ROS_ERROR_STREAM("Could not create MTS interface for CISST-ROS publisher.");
      throw std::runtime_error("Could not add function to MTS interface.");
    }

    required_interface_ = task->AddInterfaceRequired("RequiredAdapter", MTS_OPTIONAL);
    if( !required_interface_ ) {
      ROS_ERROR_STREAM("Could not create MTS interface for CISST-ROS publisher.");
      throw std::runtime_error("Could not add function to MTS interface.");
    }
  }

  void add_publisher(RosPublisherAdapter & pub_adapter)
  {
    pub_adapter.init(required_interface_, provided_interface_);
    publishers_.push_back(&pub_adapter);
  }

  mtsInterfaceRequired *required_interface_;
  mtsInterfaceProvided *provided_interface_;

  std::vector<RosPublisherAdapter*> publishers_;
  std::vector<RosSubscriberAdapter*> subscribers_;
};
         
class mtsRosTaskPeriodic : public mtsTaskPeriodic, public mtsRosTask
{
  // CISST log configuration "Run Error" by default
  CMN_DECLARE_SERVICES(CMN_NO_DYNAMIC_CREATION, CMN_LOG_LOD_RUN_ERROR);

public:
  mtsRosTaskPeriodic(const std::string & task_name, double period) :
    mtsTaskPeriodic( task_name, period, false, 500),
    mtsRosTask(this)
  { }

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
};

class mtsRosTaskFromSignal : public mtsTaskFromSignal, public mtsRosTask 
{
  // CISST log configuration "Run Error" by default
  CMN_DECLARE_SERVICES(CMN_NO_DYNAMIC_CREATION, CMN_LOG_LOD_RUN_ERROR);

public:
  mtsRosTaskFromSignal(const std::string & task_name) :
    mtsTaskFromSignal( task_name, 500),
    mtsRosTask(this)
  { }

  ~mtsRosTaskFromSignal() { };

  void Configure(const std::string & CMN_UNUSED(filename)) {};
  void Startup(void) {};
  void Cleanup(void) {};

  void Run(void)
  {
    // Process the CISST commands/events
    this->ProcessQueuedCommands();
    this->ProcessQueuedEvents();
  }
};

class mtsRosTaskFromCallback : public mtsTaskFromCallback, public mtsRosTask
{
  // CISST log configuration "Run Error" by default
  CMN_DECLARE_SERVICES(CMN_NO_DYNAMIC_CREATION, CMN_LOG_LOD_RUN_ERROR);

public:
  mtsRosTaskFromCallback(const std::string & task_name) :
    mtsTaskFromCallback( task_name, 500),
    mtsRosTask(this)
  { }

  ~mtsRosTaskFromCallback() { };

  void Configure(const std::string & CMN_UNUSED(filename)) {};
  void Startup(void) {};
  void Cleanup(void) {};

  void Run(void)
  {
    // Process the CISST commands/events
    this->ProcessQueuedCommands();
    this->ProcessQueuedEvents();
  }
};

CMN_DECLARE_SERVICES_INSTANTIATION(mtsRosTaskPeriodic);
CMN_DECLARE_SERVICES_INSTANTIATION(mtsRosTaskFromSignal);
CMN_DECLARE_SERVICES_INSTANTIATION(mtsRosTaskFromCallback);

///////////////////////////////////////////////////////////////////////////////

#endif // ifndef __CISST_ROS_INTEGRATION__ROS_TASK_H__
