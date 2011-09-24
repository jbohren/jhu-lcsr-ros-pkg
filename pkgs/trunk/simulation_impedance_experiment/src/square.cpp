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
  double volts;
  comedi_range *rangedata;
  it=comedi_open("/dev/comedi0");
  ROS_DEBUG("device: %d\n",(int)it);
  maxdata = comedi_get_maxdata(it,subdev_analog,0);
  rangedata = comedi_get_range(it,subdev_analog,0,range);
  ROS_DEBUG("maxdata: %d\n",maxdata);

  // set up ROS stuff
  ros::init(argc, argv, "square");
  ros::NodeHandle n;
  ros::Rate loop_rate(2000); // rate in Hz
  ros::Time t;
  ros::Time told;
  ros::Duration deltat;

  // loop: write out to daq

  volts = 0.5; // output 0.5 to start
  output_data = (int)((volts + 10.0)/20.0*maxdata);
  debug_msg = comedi_data_write(it,subdev_analog,chan_out,range,AREF_GROUND,output_data);

  while (ros::ok()) {

    // every 2 ms, flip the sign of the output, to get a square wave at 250 Hz

    // get timestamp and period
    t = ros::Time::now();
    deltat = t - told;
    if (deltat.toSec() >= 0.002) {
      volts = -1.0*volts;
      output_data = (int)((volts + 10)/20*maxdata);
      debug_msg = comedi_data_write(it,subdev_analog,chan_out,range,AREF_GROUND,output_data);
      told = t;
    }
    ros::spinOnce();
    loop_rate.sleep();

  }

  output_data = floor((10.0)/20.0*maxdata); // output 0 to end
  debug_msg = comedi_data_write(it,subdev_analog,chan_out,range,AREF_GROUND,output_data);

  return 0;
}
