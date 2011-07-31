
#include <cisst_ros_integration/conversions.h>

namespace cisst_ros_integration {
  /** you can explicitly define message/cisst conversions
  template<>
  std_msgs::Float64 convert(const mtsDouble & data) {
    std_msgs::Float64 msg;

    msg.data = data.GetData();

    return msg;
  }
  **/
}
