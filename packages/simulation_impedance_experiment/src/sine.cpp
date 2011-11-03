#include "ros/ros.h"
#include <comedilib.h>

const double PI = 3.14159;
int subdev_analog = 1;         // analog output is subdevice 1
int chan_out = 0;	          	 // output on channel 0
int range = 0;
int aref = AREF_GROUND;
double volts = 0;

int main(int argc, char **argv) {

  // set up comedi
  comedi_t *it;
  int debug_msg;
  int output_data;
  int maxdata;
  comedi_range *rangedata;
  it=comedi_open("/dev/comedi0");
  ROS_DEBUG("device: %d\n",(int)it);
  maxdata = comedi_get_maxdata(it,subdev_analog,0);
  rangedata = comedi_get_range(it,subdev_analog,0,range);
  ROS_DEBUG("maxdata: %d\n",maxdata);

  // set up ROS stuff
  ros::init(argc, argv, "sine");
  ros::NodeHandle n;
  ros::Rate loop_rate(2000); // rate in Hz
  ros::Time t;
  //ros::Time told;
  //ros::Duration deltat;

  // loop: write out to daq

  output_data = (int)((10.0)/20.0*maxdata); // output 0 to start
  debug_msg = comedi_data_write(it,subdev_analog,chan_out,range,AREF_GROUND,output_data);

  while (ros::ok()) {

    // get timestamp and period
    //told = t;
    t = ros::Time::now();
    //deltat = t - told;
    output_data = (int)((0.8*sin(2*PI*250*t.toSec()) + 10)/20*maxdata);
    debug_msg = comedi_data_write(it,subdev_analog,chan_out,range,AREF_GROUND,output_data);


    ros::spinOnce();
    loop_rate.sleep();

  }

  output_data = (int)((10.0)/20.0*maxdata); // output 0 to end
  debug_msg = comedi_data_write(it,subdev_analog,chan_out,range,AREF_GROUND,output_data);

  return 0;
}
