#ifndef __CISST_ROS_INTEGRATION__MTS_ROS_MSG__
#define __CISST_ROS_INTEGRATION__MTS_ROS_MSG__

// TODO: everything, basically

// ROS includes
#include <ros.h>

// CISST includes
#include <cisstMultiTask/mtsGenericObject>

template<class msg_type>
class mtsRosMsg : public mtsGenericObject {
private:
  msg_type _msg;

public:
  inline mtsRosMsg()
    : mtsGenericObject()
  {

  }

  inline mtsRosMsg(const mtsRosMsg & other):
    : mtsGenericObject(other),
    _msg(other.getMsg())
  {
  }

  ~mtsRosMsg() {

  }

  /*! Raw text output to stream */
  void ToStreamRaw(std::ostream & outputStream, const char delimiter = ' ',
      bool headerOnly = false, const std::string & headerPrefix = "") const {
    // TODO: spew something
  }

  /*! Binary serialization */
  void SerializeRaw(std::ostream & outputStream) const {
    ros::serialization::serialize(outputStream, _msg);
  }

  /*! Binary deserialization */
  void DeSerializeRaw(std::istream & inputStream) {
    ros::serialization::deserialize(outputStream, _msg);
  }
};

#endif // ifndef __CISST_ROS_INTEGRATION__MTS_ROS_MSG__
