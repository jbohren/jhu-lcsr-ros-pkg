#include "ros/ros.h"
#include "simulation_impedance_experiment/DAQinput.h"
#include "simulation_impedance_experiment/DAQoutput.h"
#include "simulation_impedance_experiment/MotorControlInfo.h"
#include "simulation_impedance_experiment/ExperimentState.h"
#include "simulation_impedance_experiment/SetHapticFeedback.h"
#include "simulation_impedance_experiment/IntegerRequest.h"
#include "simulation_impedance_experiment/keyboard.h"
#include "std_srvs/Empty.h"
#include "std_msgs/Bool.h"
#include <comedilib.h>
#include <boost/bind.hpp>

const int subdev_analog_in = 0;         // analog input is subdevice 0 for both cards
const int subdev_analog_out = 1;        // analog output is subdevice 1 for both cards
const int subdev_digital = 5;           // digital I/O is subdevice 5
const int chan_pot1 = 1;                // potentiometer 1 is on channel 1
//const int chan_pot2 = 2;                // potentiometer 2 is on channel 2
const int chan_force = 0;               // force sensor is on channel 0
const int chan_button = 0;		          // push button is on digital channel 0
const int chan_skin_stretch = 0;        // skin stretch device is on device 0, analog output channel 0
const int chan_vibro1 = 0;              // tactor 1 is on device 1, analog output channel 0
const int chan_vibro2 = 1;              // tactor 2 is on device 1, analog output channel 1
const double dev0_aout_range = 10.0;    // analog output on device 0 is +/- 10 V
const double dev1_aout_range = 5.0;     // analog output on device 1 is +/- 5 V

const double output_timeout = 0.5;    // if no output command in this period of time, set output to zero

int range_dev0_in = 0;
int range_dev0_out = 0;
int range_dev1_out = 0;
int maxdata_dev0_in;
int maxdata_dev0_out;
int maxdata_dev1_out;

// first card /dev/comedi0 for inputs and skin stretch output
comedi_t *it0;
// second card /dev/comedi1 for C-2 tactor output
comedi_t *it1; 

// feedback flags
bool vibrotactile = false;
bool skin_stretch = false;

// setpoint for skin stretch motor
// only needed when doing control loop in code, instead of using LAC board
const double zero_skin_stretch = 2.5;
double skin_stretch_setpoint = zero_skin_stretch;
double skin_stretch_setpoint_old = zero_skin_stretch;
// gains for skin stretch motor. can be changed via keyboard if appropriate flag is set in experiment manager.
double kp = 0.25;
double kd = 0.03125;
double ki = 0;//0.03125;
// error signals for PD control of skin stretch motor
double e = 0.0;
double eold = 0.0;
double ed = 0.0;
double edold = 0.0;
double eint = 0.0;
double eintold = 0.0;
double command_volts = 0.0;

// output commands (volts) for data logging
double tactor1_command = 0.0;
double tactor2_command = 0.0;
double skin_stretch_command = 0.0;


void Quit(const std_msgs::Bool::ConstPtr& msg) {
  ROS_INFO("Quit message received.");
  ros::shutdown();
}


/********************************************************************************
 * Set PD gains for skin stretch in response to signal from experiment manager.
 ********************************************************************************/

bool setPDGains(simulation_impedance_experiment::IntegerRequest::Request  &req,
                simulation_impedance_experiment::IntegerRequest::Response &res )
{
  if (req.value == KEY_7) {
    // increment kp
    kp = 2.0*kp;
  } else if (req.value == KEY_8) {
    // decrement kp
    kp = kp/2.0;
  } else if (req.value == KEY_9) {
    // increment kd
    kd = 2.0*kd;
  } else if (req.value == KEY_0) {
    // decrement kd
    kd = kd/2.0;
  } else if (req.value == KEY_I) {
    // decrement kd
    ki = 2.0*ki;
  } else if (req.value == KEY_O) {
    // decrement kd
    ki = ki/2.0;
  }
  ROS_INFO("Set skin stretch gains.\n\tkp = %f\n\tkd = %f\n\tki = %f",kp,kd,ki);
  return true;
}


/********************************************************************************
 * Set haptic feedback modes in response to information from experiment manager.
 ********************************************************************************/

bool setHapticFeedback(simulation_impedance_experiment::SetHapticFeedback::Request  &req,
                       simulation_impedance_experiment::SetHapticFeedback::Response &res )
{
  vibrotactile = req.vibrotactile;
  skin_stretch = req.skin_stretch;
  ROS_INFO("Set haptic feedback. vibro: %d, ss: %d",vibrotactile, skin_stretch);
  return true;
}


/********************************************************************************
 * Set DAQ output when haptic feedback is active.
 ********************************************************************************/

void outputCallback(const simulation_impedance_experiment::ExperimentState::ConstPtr& msg)
{
  ROS_DEBUG("Message received. Environment force = %f",msg->Fenv);

  // save timestamp of this message, so we'll always be able to tell when the last message was received
  /*
  t_last = msg->t;
  ROS_DEBUG("t_last_output_ss: %f",t_last.toSec());
  */

  /* The following is for using the Firgelli motor with the Firgelli linear actuator control board.
   * Since this board doesn't seem to be working, I'm not using this part.
  
  // set default output when skin stretch device is not active
  const double zero_skin_stretch = 1.65;

  if (skin_stretch) {
    // set min and max voltage output. note relationship is 0.066 V/mm
    const double min = 0.66;
    const double max = 2.64;
    const double scale = 0.00396;

    // convert Fenv to voltage between 0 and +3.3V
    double volts = scale*msg->Fenv + zero_skin_stretch;

    // cap outside of limits set above
    if (volts > max) {
      volts = max;
    } else if (volts < min) {
      volts = min;
    }

    // write skin stretch output to DAQ card

    // convert to number for A/D out. note that A/D range is -10V to +10V for 0 to maxdata_dev0_out.
    int output_data = (int)((volts + dev0_aout_range)/(2*dev0_aout_range)*maxdata_dev0_out);
    int debug_msg = comedi_data_write(it0,subdev_analog_out,chan_skin_stretch,range_dev0_out,AREF_GROUND,output_data);
    if (debug_msg == -1) {
      ROS_WARN("Failed to write (1) output to device 0, analog output channel %d.",chan_skin_stretch);
    }
  } else {
    int output_data = (int)((zero_skin_stretch + dev0_aout_range)/(2*dev0_aout_range)*maxdata_dev0_out);
    int debug_msg = comedi_data_write(it0,subdev_analog_out,chan_skin_stretch,range_dev0_out,AREF_GROUND,output_data);
    if (debug_msg == -1) {
      ROS_WARN("Failed to write (2) output to device 0, analog output channel %d.",chan_skin_stretch);
    }
  }

  */

  // calculate setpoint for motor.
  // The feedback potentiometer is being fed a 0 to 5 V reference, 
  // so specify the setpoint in terms of 0 to 5 V for comparison.
  
  if (skin_stretch) {
    // set limits. note the relationship is 0.1 V/mm
    const double min = 1.5;
    const double max = 3.5;
    const double scale = 0.08;
    
    // convert Fenv to voltage between 0 and +5 V
    skin_stretch_setpoint_old = skin_stretch_setpoint;
    skin_stretch_setpoint = scale*msg->Fenv + zero_skin_stretch;

    // cap outside of limits set above
    if (skin_stretch_setpoint > max) {
      skin_stretch_setpoint = max;
    } else if (skin_stretch_setpoint < min) {
      skin_stretch_setpoint = min;
    }
  } else {
    skin_stretch_setpoint = zero_skin_stretch;
  }

  if (vibrotactile) {
    // Note about hardware:
    //    Nominal input to C-2 tactors from spec sheet is sine wave at 250 Hz, 0.25 A RMS.
    //    I'm using a function generator to generate a sine wave at 250 Hz with amplitude 1.0 V RMS.
    //    This is fed into an analog multiplier chip, along with the amplitude output of the DAQ.
    //    The analog multiplier divides the product by 10.
    //    Then the resulting signal goes to a current amplifier, which puts out current = voltage / 2.
    //    So to get 0.25 A RMS out, need 0.5 V RMS, which corresponds to 5 V output from the DAQ.

    const double max = 5.0;
    const double scale = max/100.0;

    // write tactor amplitude output to DAQ card
    double volts = scale*msg->Fenv;

    // first specify which tactor. tactor 1 for positive force, tactor 2 for negative.
    int chan_on;
    int chan_off;
    if (volts >= 0) {
      chan_on = chan_vibro1;
      chan_off = chan_vibro2;
    } else {
      chan_on = chan_vibro2;
      chan_off = chan_vibro1;
      volts = -1.0*volts;
    }

    // cap voltage if needed
    if (volts > max) {
      volts = max;
    }

    // store voltage for data logging
    if (chan_on == chan_vibro1) {
      tactor1_command = volts;
      tactor2_command = 0.0;
    } else if (chan_on == chan_vibro2) {
      tactor1_command = 0.0;
      tactor2_command = volts;
    } else {
      tactor1_command = 0.0;
      tactor2_command = 0.0;
    }

    // convert to number for A/D out. note that A/D range is -10V to +10V for 0 to maxdata_dev0_out.
    int output_data = (int)((volts + dev1_aout_range)/(2*dev1_aout_range)*maxdata_dev1_out);
    int debug_msg = comedi_data_write(it1,subdev_analog_out,chan_on,range_dev1_out,AREF_DIFF,output_data);
    if (debug_msg == -1) {
      ROS_WARN("Failed to write (3a) output to device 1, analog output channel %d.",chan_on);
    }
    output_data = (int)((0.0 + dev1_aout_range)/(2*dev1_aout_range)*maxdata_dev1_out);
    debug_msg = comedi_data_write(it1,subdev_analog_out,chan_off,range_dev1_out,AREF_DIFF,output_data);
    if (debug_msg == -1) {
      ROS_WARN("Failed to write (3b) output to device 1, analog output channel %d.",chan_off);
    }
  } else {
    int output_data = (int)((0.0 + dev1_aout_range)/(2*dev1_aout_range)*maxdata_dev1_out);
    int debug_msg = comedi_data_write(it1,subdev_analog_out,chan_vibro1,range_dev1_out,AREF_DIFF,output_data);
    if (debug_msg == -1) {
      ROS_WARN("Failed to write (4) output to device 1, analog output channel %d.",chan_vibro1);
    }
    debug_msg = comedi_data_write(it1,subdev_analog_out,chan_vibro2,range_dev1_out,AREF_DIFF,output_data);
    if (debug_msg == -1) {
      ROS_WARN("Failed to write (5) output to device 1, analog output channel %d.",chan_vibro2);
    }
  }
}


/********************************************************************************
 * Skin stretch motor control
 ********************************************************************************/

void MotorControl(ros::Duration deltat, double pot1, double pot1old) {

  const double limit = 1.0; // max voltage out command. with current amp, current to motor is volts/2.
  
  // calculate error
  eold = e;
  e = pot1 - skin_stretch_setpoint;

  // simple P control
  command_volts = -1.0*kp*e;
  // add friction compensation
  /*
  if (command_volts > 0.05) {
    command_volts += 0.17;
  } else if (command_volts < -0.05) {
    command_volts -= 0.17;
  }
  */
  //ROS_INFO("setpoint: %f, actual: %f, error: %f, command: %f",skin_stretch_setpoint,pot1,e,command_volts);

  // check for hitting current limit
  if (command_volts > limit) {
    command_volts = limit;
    //ROS_WARN("Skin stretch voltage limit reached.");
  } else if (command_volts < -1.0*limit) {
    command_volts = -1.0*limit;
    //ROS_WARN("Skin stretch voltage limit reached.");
  }

  // store command (volts) for data logging)
  skin_stretch_command = command_volts;

  int output_data = (int)((command_volts + dev0_aout_range)/(2*dev0_aout_range)*maxdata_dev0_out);
  // check for hitting D/A limit
  if (output_data > maxdata_dev0_out) {
    output_data = maxdata_dev0_out;
  } else if (output_data < 0) {
    output_data = 0;
  }

  ROS_INFO("command: %f V,\toutput_data: %d\n",command_volts,output_data);
  int debug_msg = comedi_data_write(it0,subdev_analog_out,chan_skin_stretch,range_dev0_out,AREF_GROUND,output_data);
  if (debug_msg == -1) {
    ROS_WARN("Failed to write (1) output to device 0, analog output channel %d.",chan_skin_stretch);
  }

  /*
  // threshold for movement - if the difference between the pot reading and the setpoint is smaller, don't bother moving.
  const double threshold = 0.00;
  const double thresholdi = 0.0; // threshold for applying integral control
  eold = e;
  edold = ed;
  eintold = eint;
  e = pot1 - skin_stretch_setpoint;
  ed = (e - eold)/deltat.toSec();
  if (fabs(e) > thresholdi) {
    eint = eintold + e*deltat.toSec();
  }
  const double cutoff = 6; // cutoff frequency in radians per second
  double beta = exp(-1*cutoff*deltat.toSec());
  ed = beta*edold + (1.0 - beta)*ed;
  ROS_DEBUG("setpoint: %f, position: %f, error: %f",skin_stretch_setpoint,pot1,e);
  
  // simple PID control plus friction compensation
  if (fabs(e) > threshold) {
    double volts = -1.0*kp*e - kd*ed - ki*eint;
    // add compensation for Coulomb (static) friction
    if (volts > 0.05) {
      volts = volts + 0.176;
    } else if (volts < -0.05) {
      volts = volts - 0.176;
    }
    ROS_DEBUG("volts: %f",volts);
    int output_data = (int)((volts + dev0_aout_range)/(2*dev0_aout_range)*maxdata_dev0_out);
    ROS_DEBUG("output_data: %d\n",output_data);
    int debug_msg = comedi_data_write(it0,subdev_analog_out,chan_skin_stretch,range_dev0_out,AREF_GROUND,output_data);
    if (debug_msg == -1) {
      ROS_WARN("Failed to write (1) output to device 0, analog output channel %d.",chan_skin_stretch);
    }
  } else {
    int output_data = (int)((0.0 + dev0_aout_range)/(2*dev0_aout_range)*maxdata_dev0_out);
    int debug_msg = comedi_data_write(it0,subdev_analog_out,chan_skin_stretch,range_dev0_out,AREF_GROUND,output_data);
    if (debug_msg == -1) {
      ROS_WARN("Failed to write (2) output to device 0, analog output channel %d.",chan_skin_stretch);
    }
  }
  */

}


int main(int argc, char **argv) {

  double volts = 0;
  int debug_msg = 0;

  // set up comedi

  // open devices

  it0=comedi_open("/dev/comedi0");
  // want device 0 to be the pci-das6014, and device 1 to be the pci-das1602/16
  // because that's the way I hooked up the external hardware
  // check if device 0 is correct. if yes, open device 1. if no, switch, then open the other device.
  if (strcmp(comedi_get_board_name(it0),"pci-das6014") == 0) {
    ROS_INFO("device 0: %d",(int)it0);
    // open device 1
    it1=comedi_open("/dev/comedi1");
    ROS_INFO("device 1: %d",(int)it1);
  } else {
    // switch device names
    it1 = it0;
    ROS_INFO("device 0: %d",(int)it0);
    // open other device
    it0=comedi_open("/dev/comedi1");
    ROS_INFO("device 1: %d",(int)it1);
  }

  // get range and max data for analog input of device 0
  comedi_range *rangedata_dev0_in;
  maxdata_dev0_in = comedi_get_maxdata(it0,subdev_analog_in,0);
  rangedata_dev0_in = comedi_get_range(it0,subdev_analog_in,0,range_dev0_in);
  ROS_DEBUG("maxdata_dev0_in: %d",maxdata_dev0_in);

  // get range and max data for analog output of device 0
  comedi_range *rangedata_dev0_out;
  maxdata_dev0_out = comedi_get_maxdata(it0,subdev_analog_out,0);
  rangedata_dev0_out = comedi_get_range(it0,subdev_analog_out,0,range_dev0_out);
  ROS_DEBUG("maxdata_dev0_out: %d",maxdata_dev0_out);

  // configure DIO channel chan_button as output
  debug_msg = comedi_dio_config(it0,subdev_digital,chan_button,COMEDI_INPUT); // configure button input
  if (debug_msg == -1) {
    ROS_WARN("DIO failed to configure channel %d as input",chan_button);
  } else if (debug_msg == 1) {
    ROS_DEBUG("DIO successfully configured channel %d as input",chan_button);
  }
 

  // get range and max data for analog output of device 1
  comedi_range *rangedata_dev1_out;
  maxdata_dev1_out = comedi_get_maxdata(it1,subdev_analog_out,1);
  rangedata_dev1_out = comedi_get_range(it1,subdev_analog_out,1,range_dev1_out);
  ROS_DEBUG("maxdata_dev1_out: %d",maxdata_dev1_out);

  lsampl_t raw;
  unsigned int button;

  // variables to store old readings for filtering
  double Fold = 0.0;
  double pot1old = 0.0;
  //double pot2old = 0.0;


  // set up ROS stuff
  
  ros::init(argc, argv, "daq");
  ros::NodeHandle n;

  // publisher to share daq input data
  ros::Publisher daq_pub = n.advertise<simulation_impedance_experiment::DAQinput>("daq_input", 1000);

  // publisher to share daq output data
  ros::Publisher daq_out_pub = n.advertise<simulation_impedance_experiment::DAQoutput>("daq_output", 1000);

  ros::Rate loop_rate(500); // publishing rate in Hz

  // publisher to share motor control info
  ros::Publisher motor_control_pub = n.advertise<simulation_impedance_experiment::MotorControlInfo>("motor_control_info", 1000);

  // timing variables
  ros::Time t;
  ros::Time told;
  ros::Duration deltat;
  ros::Time t_buttonpress = ros::Time::now();
  /*
  ros::Time t_last_output_ss = ros::Time::now(); // stores time of most recent skin stretch output message
  ros::Time t_last_output_vt = ros::Time::now(); // stores time of most recent vibrotactile output message
  */

  // subscriber for experiment state, to get Fenv for haptic feedback output
  ros::Subscriber sub_experiment_state = n.subscribe<simulation_impedance_experiment::ExperimentState>("experiment_state", 1000,outputCallback);

  // subscriber for receiving quit command
  ros::Subscriber sub_quit = n.subscribe<std_msgs::Bool>("quit", 1000, Quit);

  // set up service for activating feedback types
  ros::ServiceServer server_set_haptic_feedback = n.advertiseService("set_haptic_feedback", setHapticFeedback);

  // set up service for changing PD gains for skin stretch
  ros::ServiceServer server_set_PD_gains = n.advertiseService("set_PD_gains", setPDGains);

  // set up service for sending button press to experiment manager
  ros::ServiceClient client_button_press = n.serviceClient<std_srvs::Empty>("handle_button_press");
  std_srvs::Empty srv_button_press;

  
  // for safety, set outputs to zero initially
  ROS_DEBUG("Initializing outputs to zero...");

  debug_msg = comedi_data_write(it0,subdev_analog_out,chan_skin_stretch,range_dev0_out,AREF_GROUND,32767);
  if (debug_msg == -1) {
    ROS_WARN("Failed to write (6) output to device 0, analog output channel %d.",chan_skin_stretch);
  }
  debug_msg = comedi_data_write(it1,subdev_analog_out,chan_vibro1,range_dev1_out,AREF_DIFF,32767);
  if (debug_msg == -1) {
    ROS_WARN("Failed to write (7) output to device 1, analog output channel %d.",chan_vibro1);
  }
  debug_msg = comedi_data_write(it1,subdev_analog_out,chan_vibro2,range_dev1_out,AREF_DIFF,32767);
  if (debug_msg == -1) {
    ROS_WARN("Failed to write (8) output to device 1, analog output channel %d.",chan_vibro2);
  }
  

  // get force sensor zero
  double zeroforce = 0;
  for (int i=0;i<500;i++) {
    debug_msg = comedi_data_read_delayed(it0,subdev_analog_in,chan_force,range_dev0_in,AREF_DIFF,&raw,3000);
    if (debug_msg == -1) {
      ROS_WARN("Read failed during force sensor calibration, force (subdev %d, chan %d)",subdev_analog_in,chan_force);
    } else {
      // convert to voltage
      volts = comedi_to_phys(raw,rangedata_dev0_in,maxdata_dev0_in);
      zeroforce = zeroforce*((double)i/((double)i+1.0)) + volts/((double)i+1.0);
    }
    ros::Duration(0.01).sleep();
  }
  ROS_INFO("zero force reading: %f V",zeroforce);


  // get initial input readings, so that once we enter the loop, we can start filtering

  t = ros::Time::now();

  // read potentiometer 1
  debug_msg = comedi_data_read_delayed(it0,subdev_analog_in,chan_pot1,range_dev0_in,AREF_GROUND,&raw,3000);
  if (debug_msg == -1) {
    ROS_WARN("Read failed, pot1 (subdev %d, chan %d)",subdev_analog_in,chan_pot1);
    pot1old = 0;
  } else {
    // convert to voltage
    pot1old = comedi_to_phys(raw,rangedata_dev0_in,maxdata_dev0_in);
    ROS_DEBUG("pot1: %f V (raw)",pot1old);
  }


  /*
  ROS_INFO("Initializing skin stretch motor...");
  // initialize skin stretch motor
  while (fabs(pot1old - skin_stretch_setpoint) > 0.05) {
    usleep(1000);
    // get timestamp and period
    told = t;
    t = ros::Time::now();
    deltat = t - told;
    // read potentiometer 1
    double pot1;
    debug_msg = comedi_data_read_delayed(it0,subdev_analog_in,chan_pot1,range_dev0_in,AREF_GROUND,&raw,3000);
    if (debug_msg == -1) {
      ROS_WARN("Read failed, pot1 (subdev %d, chan %d)",subdev_analog_in,chan_pot1);
      pot1 = 0;
    } else {
      // convert to voltage
      pot1 = comedi_to_phys(raw,rangedata_dev0_in,maxdata_dev0_in);
      ROS_DEBUG("pot1: %f V (raw)",pot1old);
    }
    MotorControl(deltat,pot1,pot1old);
    pot1old = pot1;
  }
  */
  ROS_INFO("Done.");


  // read potentiometer 2
  // not used in this experiment
  /*
  debug_msg = comedi_data_read_delayed(it0,subdev_analog_in,chan_pot2,range_dev0_in,AREF_GROUND,&raw,3000);
  if (debug_msg == -1) {
    ROS_WARN("Read failed, pot2 (subdev %d, chan %d)",subdev_analog_in,chan_pot2);
    pot2old = 0;
  } else {
    // convert to voltage
    pot2old = comedi_to_phys(raw,rangedata_dev0_in,maxdata_dev0_in);
    ROS_DEBUG("pot2: %f V (raw)",pot2old);
  }
  */
  

  // read force sensor
  debug_msg = comedi_data_read_delayed(it0,subdev_analog_in,chan_force,range_dev0_in,AREF_DIFF,&raw,3000);
  if (debug_msg == -1) {
    ROS_WARN("Read failed, force (subdev %d, chan %d)",subdev_analog_in,chan_force);
    Fold = 0;
  } else {
    // convert to voltage
    volts = comedi_to_phys(raw,rangedata_dev0_in,maxdata_dev0_in);
    // convert to force in Newtons
    Fold = (volts - zeroforce)*9.81/-0.13279;
    ROS_DEBUG("force: %f N (raw)",Fold);
  }


  // loop: read from daq and publish data; also set outputs to zero if we haven't heard anything in a while.

  while (ros::ok()) {

    // get timestamp and period
    told = t;
    t = ros::Time::now();
    deltat = t - told;


    // check to see how long it's been since we set daq output.
    // If it's been a while, the experiment manager may not be running,
    // so set the output to zero.
    
    // check for recent skin stretch output
    /*
    ROS_DEBUG("t: %f, \tt_last_output_ss: %f, \tt_last_output_vt: %f", t.toSec(),t_last_output_ss.toSec(),t_last_output_vt.toSec());
    if ((t.toSec() - t_last_output_ss.toSec()) > output_timeout) {
      // if none, set skin stretch output to zero
      ROS_INFO("No recent skin stretch output. Setting to zero.");
      debug_msg = comedi_data_write(it0,subdev_analog_out,chan_skin_stretch,range_dev0_out,AREF_GROUND,32768);
      if (debug_msg == -1) {
        ROS_WARN("Failed to write output to device 0, analog output channel %d.",chan_skin_stretch);
      }
      t_last_output_ss = t;
    }

    // check for recent tactor output
    if ((t.toSec() - t_last_output_vt.toSec()) > output_timeout) {
      // if none, set skin stretch output to zero
      ROS_INFO("No recent tactor output. Setting to zero.");
      debug_msg = comedi_data_write(it1,subdev_analog_out,chan_vibro1,range_dev1_out,AREF_GROUND,32768);
      if (debug_msg == -1) {
        ROS_WARN("Failed to write output to device 1, analog output channel %d.",chan_vibro1);
      }
      debug_msg = comedi_data_write(it1,subdev_analog_out,chan_vibro2,range_dev1_out,AREF_GROUND,32768);
      if (debug_msg == -1) {
        ROS_WARN("Failed to write output to device 1, analog output channel %d.",chan_vibro2);
      }
      t_last_output_vt = t;
    }
    */


    // declare input message type
    simulation_impedance_experiment::DAQinput msg;

    // declare output message type
    simulation_impedance_experiment::DAQoutput daq_out_msg;

    // declare motor control info message
    simulation_impedance_experiment::MotorControlInfo motor_control_msg;


    // set timestamp
    msg.t = t;
    msg.deltat = deltat;

    // read from daq and put data into the message

    // read potentiometer 1
    debug_msg = comedi_data_read_delayed(it0,subdev_analog_in,chan_pot1,range_dev0_in,AREF_GROUND,&raw,3000);
    if (debug_msg == -1) {
      ROS_WARN("Read failed, pot1 (subdev %d, chan %d)",subdev_analog_in,chan_pot1);
      msg.pot1_raw = 0;
      msg.pot1 = 0;
    } else {
      // convert to voltage
      msg.pot1_raw = comedi_to_phys(raw,rangedata_dev0_in,maxdata_dev0_in);
      // filter pot1 data
      const double cutoff = 60.0; // cutoff frequency in radians per second
      double beta = exp(-1*cutoff*deltat.toSec());
      msg.pot1 = beta*pot1old + (1.0 - beta)*msg.pot1_raw;
      ROS_DEBUG("pot1: %f V (raw), \t%f V (filtered)",msg.pot1_raw,msg.pot1);
    }

    msg.pot1_setpoint = skin_stretch_setpoint;

    // read potentiometer 2
    // not used in this experiment
    /*
    debug_msg = comedi_data_read_delayed(it0,subdev_analog_in,chan_pot2,range_dev0_in,AREF_GROUND,&raw,3000);
    if (debug_msg == -1) {
      ROS_WARN("Read failed, pot2 (subdev %d, chan %d)",subdev_analog_in,chan_pot2);
      msg.pot2_raw = 0;
      msg.pot2 = 0;
    } else {
      // convert to voltage
      msg.pot2_raw = comedi_to_phys(raw,rangedata_dev0_in,maxdata_dev0_in);
      // filter pot2 data
      const double cutoff = 6; // cutoff frequency in radians per second
      double beta = exp(-1*cutoff*deltat.toSec());
      msg.pot2 = beta*pot2old + (1.0 - beta)*msg.pot2_raw;
      ROS_DEBUG("pot2: %f V (raw), \t%f V (filtered)",msg.pot2_raw,msg.pot2);
    }
    */
    

    // read force sensor
    debug_msg = comedi_data_read_delayed(it0,subdev_analog_in,chan_force,range_dev0_in,AREF_DIFF,&raw,3000);
    if (debug_msg == -1) {
      ROS_WARN("Read failed, force (subdev %d, chan %d)",subdev_analog_in,chan_force);
      msg.force_raw = 0;
      msg.force = 0;
    } else {
      // convert to voltage
      volts = comedi_to_phys(raw,rangedata_dev0_in,maxdata_dev0_in);
      // convert to force in Newtons
      msg.force_raw = (volts - zeroforce)*9.81/-0.13279;
      // filter force data
      const double cutoff = 40.0; // cutoff frequency in radians per second
      double beta = exp(-1*cutoff*deltat.toSec());
      msg.force = beta*Fold + (1.0 - beta)*msg.force_raw;
      ROS_DEBUG("force: %f N (raw), \t%f N (filtered)",msg.force_raw,msg.force);
    }


    // read button input
    debug_msg = comedi_dio_read(it0,subdev_digital,chan_button,&button);
    if (debug_msg == -1) {
      ROS_WARN("Read failed, button (subdev %d, chan %d)",subdev_digital,chan_button);
      msg.button = false;
    } else {
      msg.button = button;
      ROS_DEBUG("button: %d",button);
    }

		// if sufficient time has passed since last button press to count this one as new and send service call to experiment manager
    if (msg.button && ((t - t_buttonpress).toSec() > 0.5)) {
      if (client_button_press.call(srv_button_press)) {
        t_buttonpress = t;
        ROS_INFO("Sent button press to experiment manager.");
      } else {
        ROS_ERROR("Failed to call service button_press.");
      }
    }

    // publish message
    daq_pub.publish(msg);


    // put daq output (volts) into a message
    daq_out_msg.tactor1 = tactor1_command;
    daq_out_msg.tactor2 = tactor2_command;
    daq_out_msg.skin_stretch = skin_stretch_command;
    daq_out_pub.publish(daq_out_msg);
    tactor1_command = 0.0;
    tactor2_command = 0.0;
    skin_stretch_command = 0.0;


    // send control signal for skin stretch motor
    // and publish motor control info
    if (skin_stretch) {
      MotorControl(deltat,msg.pot1,pot1old);
      motor_control_msg.deltat = deltat.toSec();
      motor_control_msg.setpoint = skin_stretch_setpoint;
      motor_control_msg.setpointold = skin_stretch_setpoint_old;
      motor_control_msg.pot1 = msg.pot1;
      motor_control_msg.pot1old = pot1old;
      motor_control_msg.e = e;
      motor_control_msg.eold = eold;
      motor_control_msg.ed = ed;
      motor_control_msg.edold = edold;
      motor_control_msg.eint = eint;
      motor_control_msg.eintold = eintold;
      motor_control_msg.command_volts = command_volts;
      motor_control_pub.publish(msg);
    }

    pot1old = msg.pot1;
    //pot2old = msg.pot2;
    Fold = msg.force;

    ros::spinOnce();
    loop_rate.sleep();

  }

  // set everything back to zero if node gets killed
  debug_msg = comedi_data_write(it0,subdev_analog_out,chan_skin_stretch,range_dev0_out,AREF_GROUND,32768);
  debug_msg = comedi_data_write(it1,subdev_analog_out,chan_vibro1,range_dev1_out,AREF_DIFF,32768);
  debug_msg = comedi_data_write(it1,subdev_analog_out,chan_vibro2,range_dev1_out,AREF_DIFF,32768);

  return 0;
}
