#ifndef __CISST_ROS_INTERFACE_CONVERSIONS_H__
#define __CISST_ROS_INTERFACE_CONVERSIONS_H__

#include <std_msgs/Float64.h>

#include <cisstMultiTask/mtsGenericObjectProxy.h>

namespace cisst_ros_integration {
  template<typename ROS_T, typename CISST_T>
    void cisst_to_ros(const CISST_T & data, ROS_T & msg) {
      msg.data = data.GetData();
    }

  template<typename ROS_T, typename CISST_T>
    void ros_to_cisst(const ROS_T & msg, CISST_T & data) {
      data = msg.data;
    }
}

#endif // ifndef __CISST_ROS_INTERFACE_CONVERSIONS_H__
