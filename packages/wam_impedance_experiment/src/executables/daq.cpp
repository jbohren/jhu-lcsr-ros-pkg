/****************************************************************************************
 *
 * This node reads analog and digital inputs from a DAQ card and writes analog
 * outputs to the DAQ card. In this version, there are no outputs used.
 *
 *
 * DAQ INPUT:
 *
 *   Values read from the DAQ card are converted to their physical equivalents and then
 *   put into a message (type DAQinput), broadcasted on the "daq_input" channel.
 *
 * DAQ OUTPUT:
 *
 *   No output in this version.
 *
 * RUNNING WITHOUT DAQ:
 *
 *   This ability is implemented for the purpose of testing on a computer without a DAQ
 *   card installed.
 *
 *   If the DAQ card cannot be opened, all reading and writing functions are skipped,
 *   and a flag called DAQ_active is set to false. The DAQ input values are all set to 
 *   zero (or whatever the "neutral" value is for a particular input).
 *
 ***************************************************************************************/

#include "ros/ros.h"
#include "wam_impedance_experiment/DAQinput.h"
#include "wam_impedance_experiment/ExperimentState.h"
#include "wam_impedance_experiment/Int.h"
#include "std_msgs/Empty.h"
#include <comedilib.h>

const int subdev_analog_in = 0;         // analog input is subdevice 0
const int subdev_analog_out = 1;        // analog output is subdevice 1
const int subdev_digital = 5;           // digital I/O is subdevice 5
//const int chan_pot1 = 2;                // potentiometer 1 is on channel 2 - NOT USED IN THIS EXPERIMENT
//const int chan_pot2 = 3;                // potentiometer 2 is on channel 3 - NOT USED IN THIS EXPERIMENT
const int chan_force1 = 0;               // force sensor 1 is on channel 0
const int chan_force2 = 1;               // force sensor 2 is on channel 1
//const int chan_button = 0;		          // push button is on digital channel 0 - NOT USED IN THIS EXPERIMENT
//const double aout_range = 10.0;    // analog output on device 0 is +/- 10 V - NOT USED IN THIS EXPERIMENT

int range_in = 0;
//int range_out = 0;
comedi_range *rangedata_in;
int maxdata_in;
//int maxdata_out;

comedi_t *it0;

// flag for whether there is a DAQ card or not.
bool DAQ_active = true;

// zero force readings for calibration
double zeroforce1 = 0;
double zeroforce2 = 0;

// scale factors from force sensor calibration
const double forcescale1 = 0.13541; // use negative for RH, positive for LH
const double forcescale2 = 0.13605;


void Quit(const std_msgs::Empty::ConstPtr& msg) {
  ROS_INFO("daq: DAQ: Quit message received.");
  ros::shutdown();
}


void GetZeroForce(int chan, double &zeroforce) {
  lsampl_t raw;
  double volts = 0;
  int debug_msg = 0;
  if (DAQ_active) {
    for (int i=0;i<200;i++) {
      debug_msg = comedi_data_read_delayed(it0,subdev_analog_in,chan,range_in,AREF_DIFF,&raw,3000);
      if (debug_msg == -1) {
        ROS_WARN("daq: Read failed during force sensor calibration (subdev %d, chan %d)",subdev_analog_in,chan);
      } else {
        // convert to voltage
        volts = comedi_to_phys(raw,rangedata_in,maxdata_in);
        zeroforce = zeroforce*((double)i/((double)i+1.0)) + volts/((double)i+1.0);
      }
      ros::Duration(0.001).sleep();
    }
  } else {
    ROS_WARN("daq: DAQ inactive. Skipping force sensor zero.");
  }
}

bool zeroForce(wam_impedance_experiment::Int::Request  &req,
               wam_impedance_experiment::Int::Response &res )
{
  if (req.data == 1) {
    GetZeroForce(chan_force1,zeroforce1);
  } else if (req.data == 2) {
    GetZeroForce(chan_force2,zeroforce2);
  } else {
    ROS_ERROR("daq: service zeroForce: Unknown force sensor.");
  }
  return true;
}

int main(int argc, char **argv) {

  double volts = 0;
  int debug_msg = 0;

  // set up comedi

  // open devices

  it0=comedi_open("/dev/comedi0");

  // check for successful open of device 0. if not, consider daq inactive for this run.
  if ((int)it0) {
    DAQ_active = true;
  } else {
    DAQ_active = false;
    ROS_WARN("daq: Could not open /dev/comedi0. Continuing without DAQ input.");
  }

  if (DAQ_active) {
    // get range and max data for analog input of device 0
    maxdata_in = comedi_get_maxdata(it0,subdev_analog_in,0);
    rangedata_in = comedi_get_range(it0,subdev_analog_in,0,range_in);
    ROS_DEBUG("daq: maxdata_in: %d",maxdata_in);

    // get range and max data for analog output of device 0
    // output not used in this experiment
    /*
    comedi_range *rangedata_out;
    maxdata_out = comedi_get_maxdata(it0,subdev_analog_out,0);
    rangedata_out = comedi_get_range(it0,subdev_analog_out,0,range_out);
    ROS_DEBUG("daq: maxdata_out: %d",maxdata_out);
    */

    // configure DIO channel chan_button as output
    // button not used in this experiment
    /*
    debug_msg = comedi_dio_config(it0,subdev_digital,chan_button,COMEDI_INPUT); // configure button input
    if (debug_msg == -1) {
      ROS_WARN("daq: DIO failed to configure channel %d as input",chan_button);
    } else if (debug_msg == 1) {
      ROS_DEBUG("daq: DIO successfully configured channel %d as input",chan_button);
    }
    */
   
  }

  lsampl_t raw;
  //unsigned int button;

  // variables to store old readings for filtering
  double F1old = 0.0;
  double F2old = 0.0;
  //double pot1old = 0.0;
  //double pot2old = 0.0;


  // set up ROS stuff
  
  ros::init(argc, argv, "daq");
  ros::NodeHandle n;

  // publisher to share daq input data
  ros::Publisher daq_pub = n.advertise<wam_impedance_experiment::DAQinput>("daq_input", 1000);

  ros::Rate loop_rate(1000); // publishing rate in Hz

  // timing variables
  ros::Time t;
  ros::Time told;
  ros::Duration deltat;
  ros::Time t_buttonpress = ros::Time::now();

  // subscriber for receiving quit command
  ros::Subscriber sub_quit = n.subscribe<std_msgs::Empty>("quit", 1000, Quit);

  // set up service for sending button press to experiment manager
  // button not used in this experiment
  /*
  ros::ServiceClient client_button_press = n.serviceClient<std_srvs::Empty>("handle_button_press");
  std_srvs::Empty srv_button_press;
  */
  
  // service for zeroing force sensor when requested by experiment manager
  ros::ServiceServer server_zero_force = n.advertiseService("zero_force", zeroForce);

  if (DAQ_active) {

    // for safety, set outputs to zero initially
    // outputs not used in this experiment
    /*
    ROS_DEBUG("daq: Initializing outputs to zero...");

    debug_msg = comedi_data_write(it0,subdev_analog_out,chan_skin_stretch,range_out,AREF_GROUND,32767);
    if (debug_msg == -1) {
      ROS_WARN("daq: Failed to write (6) output to device 0, analog output channel %d.",chan_skin_stretch);
    }
    */

    // get force sensor zero readings
    GetZeroForce(chan_force1,zeroforce1);
    GetZeroForce(chan_force2,zeroforce2);
    ROS_INFO("daq: zero force reading 1: %f V",zeroforce1);
    ROS_INFO("daq: zero force reading 2: %f V",zeroforce2);

    ROS_DEBUG("daq: Done.");

  }


  // get initial input readings, so that once we enter the loop, we can start filtering
  t = ros::Time::now();

  if (DAQ_active) {
    // potentiometers not used in this experiment
    /*
    // read potentiometer 1
    debug_msg = comedi_data_read_delayed(it0,subdev_analog_in,chan_pot1,range_in,AREF_GROUND,&raw,3000);
    if (debug_msg == -1) {
      ROS_WARN("daq: Read failed, pot1 (subdev %d, chan %d)",subdev_analog_in,chan_pot1);
      pot1old = 0;
    } else {
      // convert to voltage
      pot1old = comedi_to_phys(raw,rangedata_in,maxdata_in);
      ROS_DEBUG("daq: pot1: %f V (raw)",pot1old);
    }

    // read potentiometer 2
    debug_msg = comedi_data_read_delayed(it0,subdev_analog_in,chan_pot2,range_in,AREF_GROUND,&raw,3000);
    if (debug_msg == -1) {
      ROS_WARN("daq: Read failed, pot2 (subdev %d, chan %d)",subdev_analog_in,chan_pot2);
      pot2old = 0;
    } else {
      // convert to voltage
      pot2old = comedi_to_phys(raw,rangedata_in,maxdata_in);
      ROS_DEBUG("daq: pot2: %f V (raw)",pot2old);
    }
    */

    // read force sensor 1
    debug_msg = comedi_data_read_delayed(it0,subdev_analog_in,chan_force1,range_in,AREF_DIFF,&raw,3000);
    if (debug_msg == -1) {
      ROS_WARN("daq: Read failed, force (subdev %d, chan %d)",subdev_analog_in,chan_force1);
      F1old = 0;
    } else {
      // convert to voltage
      volts = comedi_to_phys(raw,rangedata_in,maxdata_in);
      // convert to force in Newtons
      F1old = (volts - zeroforce1)*9.81/forcescale1;
      ROS_DEBUG("daq: force: %f N (raw)",F1old);
    }

    // read force sensor 2
    debug_msg = comedi_data_read_delayed(it0,subdev_analog_in,chan_force2,range_in,AREF_DIFF,&raw,3000);
    if (debug_msg == -1) {
      ROS_WARN("daq: Read failed, force (subdev %d, chan %d)",subdev_analog_in,chan_force2);
      F2old = 0;
    } else {
      // convert to voltage
      volts = comedi_to_phys(raw,rangedata_in,maxdata_in);
      // convert to force in Newtons
      F2old = (volts - zeroforce2)*9.81/forcescale2;
      ROS_DEBUG("daq: force: %f N (raw)",F2old);
    }

  } else {
    
    // DAQ not active, set input values to zero
    
    //pot1old = 0.0;
    //pot2old = 0.0;
    F1old = 0.0;
    F2old = 0.0;

  }

  // declare input message type
  wam_impedance_experiment::DAQinput msg;


  // loop: read from daq and publish data; also set outputs to zero if we haven't heard anything in a while.

  while (ros::ok()) {

    // get timestamp and period
    told = t;
    t = ros::Time::now();
    deltat = t - told;


    // declare output message type
    // output not used in this experiment
    /*
    wam_impedance_experiment::DAQoutput daq_out_msg;
    */


    // set timestamp
    msg.t = t;
    msg.deltat = deltat;

    // read from daq and put data into the message

    if (DAQ_active) {
      // potentiometers not used in this experiment
      /*
      // read potentiometer 1
      debug_msg = comedi_data_read_delayed(it0,subdev_analog_in,chan_pot1,range_in,AREF_GROUND,&raw,3000);
      if (debug_msg == -1) {
        ROS_WARN("daq: Read failed, pot1 (subdev %d, chan %d)",subdev_analog_in,chan_pot1);
        msg.pot1_counts = 0;
        msg.pot1_raw = 0;
        msg.pot1 = 0;
      } else {
        msg.pot1_counts = raw;
        // convert to voltage
        msg.pot1_raw = comedi_to_phys(raw,rangedata_in,maxdata_in);
        // filter pot1 data
        const double cutoff = 60.0; // cutoff frequency in radians per second
        double beta = exp(-1*cutoff*deltat.toSec());
        msg.pot1 = beta*pot1old + (1.0 - beta)*msg.pot1_raw;
        ROS_DEBUG("daq: pot1: %f V (raw), \t%f V (filtered)",msg.pot1_raw,msg.pot1);
      }


      // read potentiometer 2
      debug_msg = comedi_data_read_delayed(it0,subdev_analog_in,chan_pot2,range_in,AREF_GROUND,&raw,3000);
      if (debug_msg == -1) {
        ROS_WARN("daq: Read failed, pot2 (subdev %d, chan %d)",subdev_analog_in,chan_pot2);
        msg.pot2_counts = 0;
        msg.pot2_raw = 0;
        msg.pot2 = 0;
      } else {
        msg.pot2_counts = raw;
        // convert to voltage
        msg.pot2_raw = comedi_to_phys(raw,rangedata_in,maxdata_in);
        // filter pot2 data
        const double cutoff = 6; // cutoff frequency in radians per second
        double beta = exp(-1*cutoff*deltat.toSec());
        msg.pot2 = beta*pot2old + (1.0 - beta)*msg.pot2_raw;
        ROS_DEBUG("daq: pot2: %f V (raw), \t%f V (filtered)",msg.pot2_raw,msg.pot2);
      }
      */

      // read force sensor 1
      debug_msg = comedi_data_read_delayed(it0,subdev_analog_in,chan_force1,range_in,AREF_DIFF,&raw,3000);
      if (debug_msg == -1) {
        ROS_WARN("daq: Read failed, force1 (subdev %d, chan %d)",subdev_analog_in,chan_force1);
        msg.force1_counts = 0;
        msg.force1_raw = 0;
        msg.force1 = 0;
      } else {
        msg.force1_counts = raw;
        // convert to voltage
        volts = comedi_to_phys(raw,rangedata_in,maxdata_in);
        // convert to force in Newtons
        msg.force1_raw = (volts - zeroforce1)*9.81/forcescale1;
        // filter force data
        const double cutoff = 20.0; // cutoff frequency in radians per second
        double beta = exp(-1*cutoff*deltat.toSec());
        msg.force1 = beta*F1old + (1.0 - beta)*msg.force1_raw;
        ROS_DEBUG("daq: force1: %f N (raw), \t%f N (filtered)",msg.force1_raw,msg.force1);
      }


      // read force sensor 2
      debug_msg = comedi_data_read_delayed(it0,subdev_analog_in,chan_force2,range_in,AREF_DIFF,&raw,3000);
      if (debug_msg == -1) {
        ROS_WARN("daq: Read failed, force2 (subdev %d, chan %d)",subdev_analog_in,chan_force2);
        msg.force2_counts = 0;
        msg.force2_raw = 0;
        msg.force2 = 0;
      } else {
        msg.force2_counts = raw;
        // convert to voltage
        volts = comedi_to_phys(raw,rangedata_in,maxdata_in);
        // convert to force in Newtons
        msg.force2_raw = (volts - zeroforce2)*9.81/forcescale2;
        // filter force data
        const double cutoff = 20.0; // cutoff frequency in radians per second
        double beta = exp(-1*cutoff*deltat.toSec());
        msg.force2 = beta*F2old + (1.0 - beta)*msg.force2_raw;
        ROS_DEBUG("daq: force2: %f N (raw), \t%f N (filtered)",msg.force2_raw,msg.force2);
      }


      // read button input
      // button not used in this experiment
      /*
      debug_msg = comedi_dio_read(it0,subdev_digital,chan_button,&button);
      if (debug_msg == -1) {
        ROS_WARN("daq: Read failed, button (subdev %d, chan %d)",subdev_digital,chan_button);
        msg.button = false;
      } else {
        msg.button = button;
        ROS_DEBUG("daq: button: %d",button);
      }

      // if sufficient time has passed since last button press to count this one as new and send service call to experiment manager
      if (msg.button && ((t - t_buttonpress).toSec() > 0.5)) {
        if (client_button_press.call(srv_button_press)) {
          t_buttonpress = t;
          ROS_INFO("daq: Sent button press to experiment manager.");
        } else {
          ROS_ERROR("daq: Failed to call service button_press.");
        }
      }
      */


    } else {
      msg.pot1_counts = 0;
      msg.pot1_raw = 0;
      msg.pot1 = 0;
      msg.pot2_counts = 0;
      msg.pot2_raw = 0;
      msg.pot2 = 0;
      msg.force1_counts = 0;
      msg.force1_raw = 0;
      msg.force1 = 0;//8*sin(2*t.toSec());
      msg.force2_counts = 0;
      msg.force2_raw = 0;
      msg.force2 = 0;//15*sin(0.5*t.toSec());
      msg.button = false;
    }

    msg.DAQactive = DAQ_active;

    msg.zeroforce1 = zeroforce1;
    msg.zeroforce2 = zeroforce2;
    msg.forcescale1 = forcescale1;
    msg.forcescale2 = forcescale2;

    // publish message
    daq_pub.publish(msg);

    // potentiometers not used in this experiment
    //pot1old = msg.pot1;
    //pot2old = msg.pot2;
    F1old = msg.force1;
    F2old = msg.force2;

    ros::spinOnce();
    loop_rate.sleep();

  }

  // set output back to zero if node gets killed
  // output not used in this experiment
  /*
  if (DAQ_active) {
    debug_msg = comedi_data_write(it0,subdev_analog_out,chan_skin_stretch,range_out,AREF_GROUND,32768);
  }
  */

  return 0;
}
