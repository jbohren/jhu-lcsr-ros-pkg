/****************************************************************************************
 *
 * This node can be used for testing other pieces of the wam impedance experiment. Its
 * primary function is to provide a keyboard interface to pass various types of
 * information to other nodes. It replaces the graphics node (which handles the keyboard
 * interface) and the experiment manager node (which specifies the various types of
 * information that are specified here by user input).
 *
 ***************************************************************************************/


#include "ros/ros.h"
#include "ros/package.h"
#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/glut.h>
#include <stdio.h>
#include "wam_impedance_experiment/key_defs.h"
#include "std_msgs/Empty.h"
#include "std_msgs/Int32.h"
#include "std_srvs/Empty.h"
#include "std_msgs/Float64MultiArray.h"
#include "wam_impedance_experiment/Int.h"
#include "wam_impedance_experiment/TaskType.h"
#include "wam_impedance_experiment/SetType.h"
#include "wam_impedance_experiment/States.h"
#include "wam_impedance_experiment/Bool.h"
#include "wam_impedance_experiment/GraphicsInfo.h"
#include "wam_impedance_experiment/ChangeState.h"
#include "wam_impedance_experiment/StartLogging.h"
#include "wam_impedance_experiment/ControllerInfo.h"
#include "wam_impedance_experiment/ControllerType.h"
#include "wam_impedance_experiment/TrajectoryInfo.h"
#include "wam_impedance_experiment/TrajectoryType.h"
#include <cisstVector/cisstVector.h>
#include <cisstCommon/cmnConstants.h>
#include "wam_impedance_experiment/robotcomponents/robManipulator_AAB.h"

const double PI = 3.14159265359;

// OpenGL and GLUT stuff in this code originally based on the tutorials at www.cs.uoi.gr/~fudos/opengl-tutorial/search.htm
// ROS stuff based on ROS beginner tutorials

ros::NodeHandle* n;
ros::Publisher* ctrl_enable_pub;
ros::Publisher* toggle_projectiles_pub;
ros::Publisher* toggle_verbose_logging_pub;
ros::Publisher* ctrl_info_pub0;
ros::Publisher* ctrl_info_pub1;
ros::Publisher* gains_pub0;
ros::Publisher* gains_pub1;
ros::Publisher* traj_info_pub0;
ros::Publisher* traj_info_pub1;
ros::ServiceClient* client_start_logging;
ros::ServiceClient* client_stop_logging;
ros::ServiceClient* client_write_data;
ros::ServiceClient* client_zero_force;

bool logging = false;
int trialNumber = 0;
int setNumber = 0;
int subjectNumber = 0;
int nSets = 0;
int nTrials = 0;
const double trialLengthPE = 15.0; // trial length for practice and experiment sets
const double trialLengthT = 60.0; // trial length for feedback training sets
double trialLength = trialLengthT;
double trialStartLength = 3.0;
stateT state = setup1;
taskType task = forcemin;
setType set_type = training;
bool debug_mode = false;
int gain_tuning_mode = 0; // 0 for off, 1 for Rn, 2 for SE3

// graphics info for drawing sine wave for tracking task.
double phase = -3.0;
double limitL = 4.0;
double limitR = 4.0;

double admittance = 0.075;
double deadband = 0.4; // in Newtons

bool change_state = false;

// robot selector
int robot = 0; // selects left wam initially

// gains for tuning (not during experiment)
std::vector<double> gains0;
std::vector<double> gains1;
int active_gain = 0;

// controller info
controllerType controller0 = gravityCompensation;
controllerType controller1 = gravityCompensation;

// pre-set setpoints
std::vector< std::vector<double> > Rn_list_0;
std::vector< std::vector<double> > Rn_list_1;
vctMatrixRotation3<double> RwU(  1,  0,  0,
                                 0,  1,  0,
                                 0,  0,  1 ); // point up (this is the initial orientation)
vctMatrixRotation3<double> RwD( -1,  0,  0,
                                 0,  1,  0,
                                 0,  0, -1 ); // point down
/*
vctMatrixRotation3<double> RwL( -1,  0,  0,
                                 0,  0, -1,
                                 0, -1,  0 ); // point to WAM left
                                 */
vctMatrixRotation3<double> RwL(  0, -1,  0,
                                 0,  0, -1,
                                 1,  0,  0 ); // point to WAM left
/*
vctMatrixRotation3<double> RwR( -1,  0,  0,
                                 0,  0,  1,
                                 0,  1,  0 ); // point to WAM right
                                 */
vctMatrixRotation3<double> RwR(  0,  1,  0,
                                 0,  0,  1,
                                 1,  0,  0 ); // point to WAM right
vctMatrixRotation3<double> RwO(  0,  0, -1,
                                 0,  1,  0,
                                 1,  0,  0 ); // point out
vctFixedSizeVector<double,3> tw(0.0,  0.0,  0.0);
std::vector< vctFrame4x4<double> > SE3_list_0;
std::vector< vctFrame4x4<double> > SE3_list_1;
std::vector<double> setpoint_null_0(7,0.0);
std::vector<double> setpoint_null_1(7,0.0);

// trajectory info
trajectoryType trajectory0 = stationary;
int direction0 = 1;
std::vector<double> Rn_0(7,0.0);
std::vector<double> Rn0_right(7,0.0);
std::vector<double> Rn0_left(7,0.0);
vctFrame4x4<double> SE3_0(RwU,tw);
unsigned int count_Rn_0 = 0;
unsigned int count_SE3_0 = 0;
trajectoryType trajectory1 = stationary;
int direction1 = 1;
std::vector<double> Rn_1(7,0.0);
vctFrame4x4<double> SE3_1(RwU,tw);
unsigned int count_Rn_1 = 0;
unsigned int count_SE3_1 = 0;


void Quit(const std_msgs::Empty::ConstPtr& msg) {
  ROS_INFO("Ctrlenabler: Quit message received.");
  ros::shutdown();
}

void ZeroForceSensor(int sensorNumber) {
  wam_impedance_experiment::Int srv_zero_force;
  srv_zero_force.request.data = sensorNumber; // specifies force sensor number 2.
  ROS_INFO("Ctrlenabler: Zeroing force sensor %d...",sensorNumber);
  if (client_zero_force->call(srv_zero_force)) {
    ROS_INFO("Ctrlenabler: Done.");
  } else {
    ROS_ERROR("Ctrlenabler: Failed to call service zero_force.");
  }
}

// Toggle projectile generation
void ToggleProjectiles() {
  std_msgs::Empty toggle_projectiles_msg;
  //ROS_INFO("Experiment manager: Toggling projectile generation.");
  toggle_projectiles_pub->publish(toggle_projectiles_msg);
}

// Toggle verbose logging
void ToggleVerboseLogging() {
  std_msgs::Empty toggle_verbose_logging_msg;
  //ROS_INFO("Experiment manager: Toggling verbose logging.");
  toggle_verbose_logging_pub->publish(toggle_verbose_logging_msg);
}

void SendPDGains() {
  std_msgs::Float64MultiArray gains_msg;
  gains_msg.data.resize(29);
  if (robot == 0) {
    gains_msg.data = gains0;
    gains_pub0->publish(gains_msg);
    ROS_INFO("Sent gains message, robot %d.",robot);
  } else if (robot == 1) {
    gains_msg.data = gains1;
    gains_pub1->publish(gains_msg);
    ROS_INFO("Sent gains message, robot %d.",robot);
  } else {
    ROS_ERROR("Unknown robot selected for gains message.");
  }
}

void SendControlInfo() {

  wam_impedance_experiment::ControllerInfo ctrl_info_msg;
  if (robot == 0) {
    ctrl_info_msg.controllerType = (int)controller0;
    ctrl_info_msg.setpoint_null.resize(7);
    ctrl_info_msg.setpoint_null = setpoint_null_0;
    ctrl_info_pub0->publish(ctrl_info_msg);
    ROS_INFO("Sent control info message, robot %d, controller %d.",robot,ctrl_info_msg.controllerType);
  } else if (robot == 1) {
    ctrl_info_msg.controllerType = (int)controller1;
    ctrl_info_msg.setpoint_null.resize(7);
    ctrl_info_msg.setpoint_null = setpoint_null_1;
    ctrl_info_pub1->publish(ctrl_info_msg);
    ROS_INFO("Sent control info message, robot %d, controller %d.",robot,ctrl_info_msg.controllerType);
  } else {
    ROS_ERROR("Unknown robot selected for control info message.");
  }

}

void SendTrajectoryInfo() {

  wam_impedance_experiment::TrajectoryInfo traj_info_msg;
  if (robot == 0) {
    traj_info_msg.trajectoryType = (int)trajectory0;
    traj_info_msg.direction = direction0;
    traj_info_msg.setpointRn = Rn_0;
    traj_info_msg.setpointSE3.layout.data_offset = 0;
    traj_info_msg.setpointSE3.layout.dim.resize(2);
    traj_info_msg.setpointSE3.layout.dim[0].stride = 4;
    traj_info_msg.setpointSE3.layout.dim[1].stride = 1;
    traj_info_msg.setpointSE3.data.resize(16);
    for (int i=0;i<4;i++) {
      for (int j=0;j<4;j++) {
        traj_info_msg.setpointSE3.data[(traj_info_msg.setpointSE3.layout.data_offset) + (traj_info_msg.setpointSE3.layout.dim[0].stride)*i + j] = SE3_0[i][j];
      }
    }
    traj_info_pub0->publish(traj_info_msg);
    ROS_INFO("Sent trajectory info message, robot %d, trajectory %d.",robot, traj_info_msg.trajectoryType);
  } else if (robot == 1) {
    traj_info_msg.trajectoryType = (int)trajectory1;
    traj_info_msg.direction = direction1;
    traj_info_msg.setpointRn = Rn_1;
    traj_info_msg.setpointSE3.layout.data_offset = 0;
    traj_info_msg.setpointSE3.layout.dim.resize(2);
    traj_info_msg.setpointSE3.layout.dim[0].stride = 4;
    traj_info_msg.setpointSE3.layout.dim[1].stride = 1;
    traj_info_msg.setpointSE3.data.resize(16);
    for (int i=0;i<4;i++) {
      for (int j=0;j<4;j++) {
        traj_info_msg.setpointSE3.data[(traj_info_msg.setpointSE3.layout.data_offset) + (traj_info_msg.setpointSE3.layout.dim[0].stride)*i + j] = SE3_1[i][j];
      }
    }
    traj_info_pub1->publish(traj_info_msg);
    ROS_INFO("Sent trajectory info message, robot %d, trajectory %d.",robot, traj_info_msg.trajectoryType);
  } else {
    ROS_ERROR("Unknown robot selected for trajectory info message.");
  }

}

void keyPressCallback(const std_msgs::Int32::ConstPtr& msg)
{
  int key = msg->data;
  ROS_INFO("Ctrlenabler: Received key press %d.",key);

  std_msgs::Empty ctrl_enable_msg;
  std_srvs::Empty srv_stop_logging;
  wam_impedance_experiment::Int srv_write_data;
  wam_impedance_experiment::StartLogging srv_start_logging;

  if ((gain_tuning_mode == 1) || (gain_tuning_mode == 2) || (gain_tuning_mode == 3)) {

    switch (key) {

      case KEY_BANG:
        // toggle gain tuning mode
        gain_tuning_mode = (gain_tuning_mode + 1) % 4;
        if (gain_tuning_mode == 3) {
          active_gain = 26;
          ROS_INFO("Gain tuning mode: nullspace/friction.");
        } else if (gain_tuning_mode == 2) {
          active_gain = 14;
          ROS_INFO("Gain tuning mode: SE3.");
        } else if (gain_tuning_mode == 1) {
          active_gain = 0;
          ROS_INFO("Gain tuning mode: Rn.");
        } else {
          ROS_INFO("Gain tuning mode: off.");
        }
        break;

      case KEY_1:
      case KEY_2:
      case KEY_3:
      case KEY_4:
      case KEY_5:
      case KEY_6:
        // activate corresponding P gain
        if (gain_tuning_mode == 1) {
          active_gain = key - 49;
        } else if (gain_tuning_mode == 2) {
          active_gain = key - 35;
        } else if (gain_tuning_mode == 3) {
          if (key == KEY_1) {
            active_gain = 26;
          } else if (key == KEY_2) {
            active_gain = 28;
          }
        }

        ROS_INFO("Active gain: %d.",active_gain);
        break;

      case KEY_7:
        // activate corresponding P gain
        if (gain_tuning_mode == 1) {
          active_gain = key - 49;
        }
        ROS_INFO("Active gain: %d.",active_gain);
        break;

      case KEY_A:
        // activate corresponding D gain
        if (gain_tuning_mode == 1) {
          active_gain = 7;
        } else if (gain_tuning_mode == 2) {
          active_gain = 20;
        } else if (gain_tuning_mode == 3) {
          active_gain = 27;
        }
        ROS_INFO("Active gain: %d.",active_gain);
        break;

      case KEY_S:
        // activate corresponding D gain
        if (gain_tuning_mode == 1) {
          active_gain = 8;
        } else if (gain_tuning_mode == 2) {
          active_gain = 21;
        }
        ROS_INFO("Active gain: %d.",active_gain);
        break;

      case KEY_D:
        // activate corresponding D gain
        if (gain_tuning_mode == 1) {
          active_gain = 9;
        } else if (gain_tuning_mode == 2) {
          active_gain = 22;
        }
        ROS_INFO("Active gain: %d.",active_gain);
        break;

      case KEY_F:
        // activate corresponding D gain
        if (gain_tuning_mode == 1) {
          active_gain = 10;
        } else if (gain_tuning_mode == 2) {
          active_gain = 23;
        }
        ROS_INFO("Active gain: %d.",active_gain);
        break;

      case KEY_G:
        // activate corresponding D gain
        if (gain_tuning_mode == 1) {
          active_gain = 11;
        } else if (gain_tuning_mode == 2) {
          active_gain = 24;
        }
        ROS_INFO("Active gain: %d.",active_gain);
        break;

      case KEY_H:
        // activate corresponding D gain
        if (gain_tuning_mode == 1) {
          active_gain = 12;
        } else if (gain_tuning_mode == 2) {
          active_gain = 25;
        }
        ROS_INFO("Active gain: %d.",active_gain);
        break;

      case KEY_J:
        // activate corresponding D gain
        if (gain_tuning_mode == 1) {
          active_gain = 13;
        }
        ROS_INFO("Active gain: %d.",active_gain);
        break;

      case KEY_PLUS:
      case KEY_EQUAL:
        if (robot == 0) {
          gains0[active_gain] = 1.1*gains0[active_gain];
        } else if (robot == 1) {
          gains1[active_gain] = 1.1*gains1[active_gain];
        }
        SendPDGains();
        break;

      case KEY_MINUS:
      case KEY_US:
        if (robot == 0) {
          gains0[active_gain] = gains0[active_gain]/1.1;
        } else if (robot == 1) {
          gains1[active_gain] = gains1[active_gain]/1.1;
        }
        SendPDGains();
        break;

      case KEY_W:
        // start or stop data logging
        if (logging) {
          // stop logging
          ROS_INFO("calling log stop service");
          if (client_stop_logging->call(srv_stop_logging)) {
            ROS_INFO("Stopped logging.");
          } else {
            ROS_ERROR("Failed to call service start_logging.");
          }
          ROS_INFO("finished calling log stop service");
          srv_write_data.request.data = 3;
          ROS_INFO("calling write data service");
          if (client_write_data->call(srv_write_data)) {
            ROS_INFO("Wrote data file.");
          } else {
            ROS_ERROR("Failed to call service write_data.");
          }
          ROS_INFO("finished calling write data service");
          trialNumber++;
        } else {
          // start logging
          srv_start_logging.request.task = (int)task;
          srv_start_logging.request.subjectNumber = subjectNumber;
          srv_start_logging.request.trialNumber = trialNumber;
          srv_start_logging.request.setNumber = setNumber;
          srv_start_logging.request.set_type = (int)set_type;
          srv_start_logging.request.direction = direction1; // robot 1 is the one under computer control for the experiment
          srv_start_logging.request.admittance = admittance;
          srv_start_logging.request.deadband = deadband;
          if (client_start_logging->call(srv_start_logging)) {
            ROS_INFO("Started logging.");
          } else {
            ROS_ERROR("Failed to call service start_logging.");
          }
        }
        logging = !logging;
        break;

      case KEY_L:
        // select left wam
        robot = 0;
        ROS_INFO("Robot %d active.",robot);
        break;

      case KEY_R:
        // select right wam
        robot = 1;
        ROS_INFO("Robot %d active.",robot);
        break;

      default:
        ROS_INFO("Pressing %d doesn't do anything.",key);
        break;

    }

  } else {

    switch (key) {

    case KEY_Z:
      // zero user input force sensor
      if (debug_mode) {
        ZeroForceSensor(1);
        debug_mode = false;
      }
      break;

    case KEY_X:
      // zero environment force sensor
      if (debug_mode) {
        ZeroForceSensor(2);
        debug_mode = false;
      }
      break;


    case KEY_V:
      // toggle verbose logging
      if (debug_mode) {
        ToggleVerboseLogging();
        debug_mode = false;
      }
      break;


      case KEY_TILDE:
        // toggle debug mode
        debug_mode = !debug_mode;
        break;

      case KEY_BANG:
        // toggle gain tuning mode
        gain_tuning_mode = (gain_tuning_mode + 1) % 4;
        if (gain_tuning_mode == 3) {
          ROS_INFO("Gain tuning mode: nullspace/friction.");
        } else if (gain_tuning_mode == 2) {
          ROS_INFO("Gain tuning mode: SE3.");
        } else if (gain_tuning_mode == 1) {
          ROS_INFO("Gain tuning mode: Rn.");
        } else {
          ROS_INFO("Gain tuning mode: off.");
        }
        break;

      case KEY_SPC:
      case KEY_ENTER:
        // advance state
        if (state == cleanup4) {
          state = setup1;
        } else {
          state = (stateT)((int)state + 1);
        }
        ROS_INFO("Ctrlenabler: Advanced state - %d.",(int)state);
        change_state = true;
        break;

      case KEY_T:
        // switch task
        if (task == forcemin) {
          task = tracking;
          ROS_INFO("Ctrlenabler: Activated tracking task.");
        } else if (task == tracking) {
          task = forcemin;
          ROS_INFO("Ctrlenabler: Activated forcemin task.");
        } else {
          ROS_ERROR("Ctrlenabler: Unknown task.");
        }
        change_state = true;
        break;

      case KEY_Y:
        // switch set type
        if (set_type == training) {
          set_type = practice;
          ROS_INFO("Ctrlenabler: Switched set type to practice.");
        } else if (set_type == practice) {
          set_type = experiment;
          ROS_INFO("Ctrlenabler: Switched set type to experiment.");
        } else if (set_type == experiment) {
          set_type = training;
          ROS_INFO("Ctrlenabler: Switched set type to training.");
        } else {
          ROS_ERROR("Ctrlenabler: Unknown set type.");
        }
        change_state = true;
        break;

      case KEY_L:
        // select left wam
        robot = 0;
        ROS_INFO("Robot %d active.",robot);
        break;

      case KEY_R:
        // select right wam
        robot = 1;
        ROS_INFO("Robot %d active.",robot);
        break;

      case KEY_P:
        // start or stop projectiles
        if (debug_mode) {
          ToggleProjectiles();
          debug_mode = false;
        }
        break;

      case KEY_W:
        // start or stop data logging
        if (logging) {
          // stop logging
          ROS_INFO("calling log stop service");
          if (client_stop_logging->call(srv_stop_logging)) {
            ROS_INFO("Stopped logging.");
          } else {
            ROS_ERROR("Failed to call service start_logging.");
          }
          ROS_INFO("finished calling log stop service");
          srv_write_data.request.data = 3;
          ROS_INFO("calling write data service");
          if (client_write_data->call(srv_write_data)) {
            ROS_INFO("Wrote data file.");
          } else {
            ROS_ERROR("Failed to call service write_data.");
          }
          ROS_INFO("finished calling write data service");
          trialNumber++;
        } else {
          // start logging
          srv_start_logging.request.task = (int)task;
          srv_start_logging.request.subjectNumber = subjectNumber;
          srv_start_logging.request.trialNumber = trialNumber;
          srv_start_logging.request.setNumber = setNumber;
          srv_start_logging.request.set_type = (int)set_type;
          srv_start_logging.request.direction = direction1; // robot 1 is the one under computer control for the experiment
          srv_start_logging.request.admittance = admittance;
          srv_start_logging.request.deadband = deadband;
          if (client_start_logging->call(srv_start_logging)) {
            ROS_INFO("Started logging.");
          } else {
            ROS_ERROR("Failed to call service start_logging.");
          }
        }
        logging = !logging;
        break;

      case KEY_E:

        // send PD gains
        if (robot == 0) {
          
          robot = 1;
          SendPDGains();
          robot = 0;
          SendPDGains();

          // send controller enable message
          ctrl_enable_pub->publish(ctrl_enable_msg);
          ROS_INFO("Sent controller enable message.");

        } else if (robot == 1) {
          
          robot = 0;
          SendPDGains();
          robot = 1;
          SendPDGains();
          
          // send controller enable message
          ctrl_enable_pub->publish(ctrl_enable_msg);
          ROS_INFO("Sent controller enable message.");

        } else {

          ROS_ERROR("Unknown robot number. Controller enable signal not sent.");

        }

        break;

      case KEY_1:
      case KEY_2:
      case KEY_3:
      case KEY_4:
      case KEY_5:
        // send trajectory select message
        if (robot == 0) {
          trajectory0 = trajectoryType(key - 48);
        } else if (robot == 1) {
          trajectory1 = trajectoryType(key - 48);
        }
        SendTrajectoryInfo();
        break;

      case KEY_6:
      case KEY_7:
      case KEY_8:
      case KEY_9:
      case KEY_0:
        // send control select message
        if (robot == 0) {
          if (key == KEY_0) {
            controller0 = controllerType(5);
          } else {
            controller0 = controllerType(key - 53);
          }
        } else if (robot == 1) {
          if (key == KEY_0) {
            controller1 = controllerType(5);
          } else {
            controller1 = controllerType(key - 53);
          }
        }
        SendControlInfo();
        break;

      case KEY_PLUS:
      case KEY_EQUAL:
        if (robot == 0) {
          direction0 = 1;
        } else if (robot == 1) {
          direction1 = 1;
        }
        SendControlInfo();
        SendTrajectoryInfo();
        break;

      case KEY_MINUS:
      case KEY_US:
        if (robot == 0) {
          direction0 = -1;
        } else if (robot == 1) {
          direction1 = -1;
        }
        SendControlInfo();
        SendTrajectoryInfo();
        break;

      case KEY_N:
        // next setpoint
        if (robot == 0) {
          if ((trajectory0 == setpointsRn) && (count_Rn_0 < Rn_list_0.size())) {
            Rn_0 = Rn_list_0[count_Rn_0];
            count_Rn_0++;
            if (count_Rn_0 == Rn_list_0.size()) {
              count_Rn_0 = 0;
            }
          } else if ((trajectory0 == setpointsSE3) && (count_SE3_0 < SE3_list_0.size())) {
            SE3_0 = SE3_list_0[count_SE3_0];
            count_SE3_0++;
            if (count_SE3_0 == SE3_list_0.size()) {
              count_SE3_0 = 0;
            }
          }
        } else if (robot == 1) {
          if ((trajectory1 == setpointsRn) && (count_Rn_1 < Rn_list_1.size())) {
            Rn_1 = Rn_list_1[count_Rn_1];
            count_Rn_1++;
            if (count_Rn_1 == Rn_list_1.size()) {
              count_Rn_1 = 0;
            }
          } else if ((trajectory1 == setpointsSE3) && (count_SE3_1 < SE3_list_1.size())) {
            SE3_1 = SE3_list_1[count_SE3_1];
            count_SE3_1++;
            if (count_SE3_1 == SE3_list_1.size()) {
              count_SE3_1 = 0;
            }
          }
        }
        SendTrajectoryInfo();
        break;

      default:
        ROS_INFO("Pressing %d doesn't do anything.",key);
        break;

    }

  }
}


int main(int argc, char **argv) {

  // set up ROS stuff
  ros::init(argc, argv, "ctrl_enabler");
  n = new ros::NodeHandle;
  ros::Rate loop_rate(100);

  n->setParam("/admittance",admittance);
  n->setParam("/deadband",deadband);

  ctrl_enable_pub = new ros::Publisher;
  *ctrl_enable_pub = n->advertise<std_msgs::Empty>("ctrl_enable",100);

  ctrl_info_pub0 = new ros::Publisher;
  *ctrl_info_pub0 = n->advertise<wam_impedance_experiment::ControllerInfo>("ctrl_info_0",100);

  ctrl_info_pub1 = new ros::Publisher;
  *ctrl_info_pub1 = n->advertise<wam_impedance_experiment::ControllerInfo>("ctrl_info_1",100);

  gains_pub0 = new ros::Publisher;
  *gains_pub0 = n->advertise<std_msgs::Float64MultiArray>("gains_0",100);
  gains0.resize(29);
  // P gains Rn          // D gains Rn
  gains0[0] = 800;       gains0[7] = 12.0;
  gains0[1] = 800;       gains0[8] = 6.0;
  gains0[2] = 750;       gains0[9] = 10.0;
  gains0[3] = 625;       gains0[10] = 5.0;
  gains0[4] = 3500;      gains0[11] = 40.0;
  gains0[5] = 4000;      gains0[12] = 25.0;
  gains0[6] = 8000;      gains0[13] = 30.0;
  // P gains SE3         // D gains SE3
  gains0[14] = 1000;     gains0[20] = 10.0;
  gains0[15] = 1000;     gains0[21] = 10.0;
  gains0[16] = 1000;     gains0[22] = 10.0;
  gains0[17] = 5.5;      gains0[23] = 0.1;
  gains0[18] = 1.0;      gains0[24] = 0.1;
  gains0[19] = 4.0;      gains0[25] = 0.1;
  // P gain nullspace    // D gain nullspace
  gains0[26] = 1.0;     gains0[27] = 0.15;
  // friction gain
  gains0[28] = 0.5;

  gains_pub1 = new ros::Publisher;
  *gains_pub1 = n->advertise<std_msgs::Float64MultiArray>("gains_1",100);
  gains1.resize(29);
  // P gains Rn          // D gains Rn
  gains1[0] = 800;       gains1[7] = 12.0;
  gains1[1] = 800;       gains1[8] = 6.0;
  gains1[2] = 750;       gains1[9] = 10.0;
  gains1[3] = 625;       gains1[10] = 5.0;
  gains1[4] = 3500;      gains1[11] = 40.0;
  gains1[5] = 675;       gains1[12] = 7.5;
  gains1[6] = 8000;      gains1[13] = 30.0;
  // P gains SE3         // D gains SE3
  gains1[14] = 1000;     gains1[20] = 10.0;
  gains1[15] = 100;      gains1[21] = 1.0;
  gains1[16] = 1000;     gains1[22] = 10.0;
  gains1[17] = 5.5;      gains1[23] = 0.1;
  gains1[18] = 1.0;      gains1[24] = 0.1;
  gains1[19] = 4.0;      gains1[25] = 0.1;
  // P gain nullspace    // D gain nullspace
  gains1[26] = 0.4;      gains1[27] = 0.1;
  // friction gain
  gains1[28] = 0.5;

  traj_info_pub0 = new ros::Publisher;
  *traj_info_pub0 = n->advertise<wam_impedance_experiment::TrajectoryInfo>("traj_info_0",100);

  traj_info_pub1 = new ros::Publisher;
  *traj_info_pub1 = n->advertise<wam_impedance_experiment::TrajectoryInfo>("traj_info_1",100);

  client_start_logging = new ros::ServiceClient;
  *client_start_logging = n->serviceClient<wam_impedance_experiment::StartLogging>("start_logging");

  client_stop_logging = new ros::ServiceClient;
  *client_stop_logging = n->serviceClient<std_srvs::Empty>("stop_logging");

  client_write_data = new ros::ServiceClient;
  *client_write_data = n->serviceClient<wam_impedance_experiment::Int>("write_data");

  client_zero_force = new ros::ServiceClient;
  *client_zero_force = n->serviceClient<wam_impedance_experiment::Int>("zero_force");

  // subscriber for receiving key presses
  ros::Subscriber sub_key_press = n->subscribe<std_msgs::Int32>("key_press", 1000, keyPressCallback);

  // subscriber for receiving quit command
  ros::Subscriber sub_quit = n->subscribe<std_msgs::Empty>("quit", 1000, Quit);

  // publishers for communicating with graphics node
  ros::Publisher graphics_info_pub = n->advertise<wam_impedance_experiment::GraphicsInfo>("graphics_info", 1000);
  wam_impedance_experiment::GraphicsInfo msg_graphics_info;

  // services for communicating with graphics node
  ros::ServiceClient client_set_graphics_mode = n->serviceClient<wam_impedance_experiment::Bool>("set_graphics_mode");
  ros::ServiceClient client_change_state = n->serviceClient<wam_impedance_experiment::ChangeState>("change_state");
  wam_impedance_experiment::Bool srv_set_graphics_mode;
  wam_impedance_experiment::ChangeState srv_change_state;

  toggle_projectiles_pub = new ros::Publisher;
  *toggle_projectiles_pub = n->advertise<std_msgs::Empty>("toggle_projectiles",100);

  toggle_verbose_logging_pub = new ros::Publisher;
  *toggle_verbose_logging_pub = n->advertise<std_msgs::Empty>("toggle_verbose_logging",100);

  // initial robot position
  vctDynamicVector<double> qinit(7, 0.0);
  qinit[1] = -cmnPI_2;
  qinit[3] =  cmnPI;

  // get initial end effector position and orientation in world frame
  vctMatrixRotation3<double> Rw0(
      0.0,  0.0, -1.0, 
      0.0,  1.0,  0.0, 
      1.0,  0.0,  0.0 );
  vctFixedSizeVector<double,3> tw0(0.0);
  vctFrame4x4<double> Rtw0( Rw0, tw0 );
  std::string robfile("libs/etc/cisstRobot/WAM/wam7.rob");
  robfile = ros::package::getPath("cisst")+"/build/source/"+robfile;
  robManipulator_AAB WAMtmp( robfile, Rtw0 );
  vctFrame4x4<double> Rtwninit = WAMtmp.ForwardKinematics( qinit );

  // set initial setpoint values to initial pose
  for (unsigned int i=0;i<qinit.size();i++) {
    Rn_0[i] = qinit[i];
    Rn_1[i] = qinit[i];
  }
  SE3_0 = Rtwninit;
  SE3_1 = Rtwninit;
  tw = Rtwninit.Translation();

  // set nullspace setpoint
  //setpoint_null_0[0] = 0.6;
  //setpoint_null_0[1] = -0.76;
  setpoint_null_0[2] = 0.8;
  //setpoint_null_0[2] = 0.9713;
  setpoint_null_1[2] = -0.8;

  // make setpoint lists

  vctFrame4x4<double> SE3_1(RwU,tw);
  tw[0] = -0.85;
  tw[2] = -0.5;
  vctFrame4x4<double> SE3_2(RwU,tw);
  //tw[0] = -2.0; // should be outside workspace
  //tw[1] = -0.4;
  //vctFrame4x4<double> SE3_3(RwU,tw);
  tw[0] = -0.85;
  vctFrame4x4<double> SE3_4_0(RwR,tw);
  vctFrame4x4<double> SE3_4_1(RwL,tw);
  //tw[1] = 0.2;
  vctFrame4x4<double> SE3_5(RwU,tw);
  SE3_list_0.push_back( SE3_1 );
  SE3_list_0.push_back( SE3_2 );
  //SE3_list_0.push_back( SE3_3 );
  SE3_list_0.push_back( SE3_4_0 );
  SE3_list_0.push_back( SE3_5 );
 
  SE3_list_1.push_back( SE3_1 );
  SE3_list_1.push_back( SE3_2 );
  //SE3_list_1.push_back( SE3_3 );
  SE3_list_1.push_back( SE3_4_1 );
  SE3_list_1.push_back( SE3_5 );
  
 

  std::vector<double> Rn0(Rn_0);
  std::vector<double> Rn1(Rn_1);
  Rn_list_0.push_back(Rn0);
  Rn_list_1.push_back(Rn1);
  Rn0[0] = 0.85200066108372685;
  Rn0[1] = -0.71117053781398032;
  Rn0[2] = 1.4398269787074245;
  Rn0[3] = 1.4624115427739803;
  Rn0[4] = -0.70173978609938803;
  Rn0[5] = 0.83672971930812079;
  Rn0[6] = -0.22644661266545243;
  Rn1[0] = -0.84352779773070785;
  Rn1[1] = -0.74601279380506202;
  Rn1[2] = -1.3807088353951313;
  Rn1[3] = 1.4624115427739641;
  Rn1[4] = 0.61414993604158652;
  Rn1[5] = 0.79927303718499287;
  Rn1[6] = 0.29283240889511913;
  /*
  Rn0[0] = 0.839125;
  Rn0[1] = -0.760694;
  Rn0[2] = 1.356141;
  Rn0[3] = 1.4624115;
  Rn0[4] = -0.574992;
  Rn0[5] = 0.784711;
  Rn0[6] = -1.893884;
  Rn1[0] = -0.839125;
  Rn1[1] = -0.760694;
  Rn1[2] = -1.356141;
  Rn1[3] = 1.4624115;
  Rn1[4] = 0.574992;
  Rn1[5] = 0.784711;
  Rn1[6] = 1.893884;
  */
  /*
  Rn0[0] = 0.93492;
  Rn0[1] = -1.02947;
  Rn0[2] = 0.9713;
  Rn0[3] = 0.51965;
  Rn0[4] = -0.00737;
  Rn0[5] = 1.81199;
  Rn0[6] = -2.35728;
  Rn1[0] = -0.93492;
  Rn1[1] = -1.02947;
  Rn1[2] = -0.9713;
  Rn1[3] = 0.51965;
  Rn1[4] = 0.00737;
  Rn1[5] = 1.81199;
  Rn1[6] = 2.35728;
  */
  Rn_list_0.push_back(Rn0);
  Rn_list_1.push_back(Rn1);

  // right and left workspace limits for graphics calibration
  Rn0_right[0] = 0.42962376104006644;
  Rn0_right[1] = -0.65977407859702486;
  Rn0_right[2] = 1.5719315853255198;
  Rn0_right[3] = 1.4316313261758604;
  Rn0_right[4] = -0.67095457841031703;
  Rn0_right[5] = 0.47063098789494923;
  Rn0_right[6] = -0.21247625878308049;
  Rn0_left[0] = 1.066115581463873;
  Rn0_left[1] = -0.81360642826702789;
  Rn0_left[2] = 1.4472279624755078;
  Rn0_left[3] = 1.2178964987427359;
  Rn0_left[4] = -0.56124585921046288;
  Rn0_left[5] = 1.2374593364024837;
  Rn0_left[6] = -0.27905489306327208;

  Rn_list_0.push_back(Rn0_right);
  Rn_list_0.push_back(Rn0_left);

  while (ros::ok()) {

    ros::Time t = ros::Time::now();

    // if state change, tell graphics node
    if (change_state) {
      srv_change_state.request.state = (int)state;
      srv_change_state.request.task = (int)task;
      srv_change_state.request.trial_number = trialNumber;
      srv_change_state.request.nTrials = nTrials;
      srv_change_state.request.trial_length = trialLength;
      srv_change_state.request.set_type = int(set_type);
      if (client_change_state.call(srv_change_state)) {
        ROS_INFO("State changed.");
      } else {
        ROS_ERROR("Failed to call service change_state.");
      }
      change_state = false;
    }

    // if debug mode changed, tell graphics node
    if (srv_set_graphics_mode.request.data != debug_mode) {
      srv_set_graphics_mode.request.data = debug_mode;
      if (client_set_graphics_mode.call(srv_set_graphics_mode)) {
        ROS_INFO("Graphics mode set.");
      } else {
        ROS_ERROR("Failed to call service set_graphics_mode.");
      }
    }

    if ((state == runningTrial) && (task == tracking)) {
      // put info for drawing sine wave into graphics info message
      limitL = -4.0;
      limitR = 4.0;
      phase = t.toSec();
      msg_graphics_info.t = t;
      msg_graphics_info.direction = direction0; // robot 0 is the user-controlled robot
      msg_graphics_info.limitL = limitL;
      msg_graphics_info.limitR = limitR;
      msg_graphics_info.phase = phase;
      graphics_info_pub.publish(msg_graphics_info);
    }

    ros::spinOnce();
    loop_rate.sleep();

  }

  delete ctrl_enable_pub;
  delete toggle_projectiles_pub;
  delete toggle_verbose_logging_pub;
  delete ctrl_info_pub0;
  delete ctrl_info_pub1;
  delete gains_pub0;
  delete gains_pub1;
  delete traj_info_pub0;
  delete traj_info_pub1;
  delete client_start_logging;
  delete client_stop_logging;
  delete client_write_data;
  delete client_zero_force;
  delete n;

  return 0;
}
