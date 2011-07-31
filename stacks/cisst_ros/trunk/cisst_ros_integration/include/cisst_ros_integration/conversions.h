#ifndef __CISST_ROS_INTERFACE_CONVERSIONS_H__
#define __CISST_ROS_INTERFACE_CONVERSIONS_H__

#include <std_msgs/Float64.h>

#include <cisstMultiTask/mtsGenericObjectProxy.h>

namespace cisst_ros_integration {
  template<typename ROS_T, typename MTS_T>
    ROS_T ros_from_mts(const MTS_T & data) {
      ROS_T msg;

      msg.data = data.GetData();

      return msg;
    }

  template<typename ROS_T, typename MTS_T>
    MTS_T mts_from_ros(const ROS_T & msg) {

      MTS_T data(msg.data);

      return data;
    }
}

#endif // ifndef __CISST_ROS_INTERFACE_CONVERSIONS_H__
