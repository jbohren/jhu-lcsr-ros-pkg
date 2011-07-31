#ifndef __CISST_ROS_INTERFACE_CONVERSIONS_H__
#define __CISST_ROS_INTERFACE_CONVERSIONS_H__

#include <std_msgs/Float64.h>

#include <cisstMultiTask/mtsGenericObjectProxy.h>

namespace cisst_ros_integration {
  template<typename MSG_T, typename CISST_T>
    MSG_T convert(const CISST_T & data) {
      MSG_T msg;

      msg.data = data.GetData();

      return msg;
    }
}

#endif // ifndef __CISST_ROS_INTERFACE_CONVERSIONS_H__
