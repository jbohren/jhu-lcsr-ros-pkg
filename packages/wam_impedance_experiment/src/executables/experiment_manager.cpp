/****************************************************************************************
 *
 * This node controls the progression of the experiment through different sets and
 * trials. It tells the other nodes what to do:
 *    starts and stops logging
 *    passes info to the graphics node telling it what to draw
 *    tells the cisst node which controller to use
 *    tells the cisst node which trajectory to follow, and which setpoints to use
 *    sets controller gains
 *
 ***************************************************************************************/

#include <stdio.h>
#include <fstream>
#include "ros/ros.h"
#include "ros/package.h"
#include "wam_impedance_experiment/key_defs.h"
#include "wam_impedance_experiment/GraphicsInfo.h"
#include "wam_impedance_experiment/ChangeState.h"
#include "wam_impedance_experiment/TaskType.h"
#include "wam_impedance_experiment/SetType.h"
#include "wam_impedance_experiment/States.h"
//#include "wam_impedance_experiment/Gains.h"
#include "wam_impedance_experiment/Bool.h"
#include "std_msgs/Empty.h"
#include "std_srvs/Empty.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Float64MultiArray.h"
#include "wam_impedance_experiment/StartLogging.h"
#include "wam_impedance_experiment/ControllerInfo.h"
#include "wam_impedance_experiment/ControllerType.h"
#include "wam_impedance_experiment/TrajectoryInfo.h"
#include "wam_impedance_experiment/TrajectoryType.h"
#include "wam_impedance_experiment/Int.h"
#include <cisstVector/cisstVector.h>
#include <cisstCommon/cmnConstants.h>
#include "wam_impedance_experiment/robotcomponents/robManipulator_AAB.h"

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
ros::ServiceClient* client_change_state;

bool logging = false;
bool debug_mode = false;
bool auto_run = false;
ros::Time* t_start;

// admittance and deadband for user input
double admittance = 0.075;
double deadband = 0.4; // in Newtons

// robot selector
int robot = 0; // selects left wam initially

// graphics info for drawing sine wave for tracking task.
double phase = -2.1;
double limitL = 4.0;
double limitR = 4.0;

// flags for set type
setType set_type = practice;
bool variable = false; // variable vs. constant impedance
taskType task = forcemin;

// structure to hold impedance info
typedef struct {
	double k;
	double b;
} impedance;

// gains for user and moving object
impedance imp1;
impedance imp2;

// gains for tuning (not during experiment)
std::vector<double> gains0;
std::vector<double> gains1;
int active_gain = 0;

// controller info
controllerType controller0 = gravityCompensation;
controllerType controller1 = gravityCompensation;

// time in seconds to wait for controller transition. 
// IMPORTANT: **make sure this is the same in the controller**
double controllerTransitionTime = 3.0; 

// pre-set setpoints
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
std::vector<double> setpoint_null_0(7,0.0);
std::vector<double> setpoint_null_1(7,0.0);

// trajectory info
trajectoryType trajectory0 = stationary;
std::vector<double> Rn_0(7,0.0);
vctFrame4x4<double> SE3_0(RwU,tw);
trajectoryType trajectory1 = stationary;
int direction0 = 1;
std::vector<double> Rn_1(7,0.0);
vctFrame4x4<double> SE3_1(RwU,tw);
int direction1 = 1;
std::vector<double> Rn0_start(7,0.0);
std::vector<double> Rn1_start(7,0.0);
std::vector<double> Rn_home(7,0.0);
std::vector<double> Rn0_right(7,0.0);
std::vector<double> Rn0_left(7,0.0);
std::vector<double> Rn0_current(7,0.0);
std::vector<double> Rn1_current(7,0.0);


const int MAX_N_SETS = 50;
const int MAX_N_TRIALS = 100;

// structure for storing trial info
typedef struct {
  float k1[MAX_N_TRIALS];
  float b1[MAX_N_TRIALS];
  float direction[MAX_N_TRIALS];
  int nTrials;        // number of trials in this set
  int task;           // task type: forcemin or tracking
  int set_type;       // practice or experiment
  float trialLength;  // trial length in seconds
} trialInfo;

trialInfo trials[MAX_N_SETS];

int subjectNumber = 0;
int nSets = 0;
int nTrials = 0;
int setNumber = 1;
int trialNumber = 1;
double trialLength = 15.0;
double trialStartLength = 2.1;
int user_rating = 3;

// experiment states
stateT state = setup1;

// path for getting trial info from file
std::string pathname;

void Quit(const std_msgs::Empty::ConstPtr& msg) {
  ROS_INFO("Experiment manager: Quit message received.");
  ros::shutdown();
}

void ZeroForceSensor(int sensorNumber) {
  wam_impedance_experiment::Int srv_zero_force;
  srv_zero_force.request.data = sensorNumber; // specifies force sensor number 2.
  ROS_INFO("Experiment manager: Zeroing force sensor %d...",sensorNumber);
  if (client_zero_force->call(srv_zero_force)) {
    ROS_INFO("Experiment manager: Done.");
  } else {
    ROS_ERROR("Experiment manager: Failed to call service zero_force.");
  }
}

void SendGains() {
  std_msgs::Float64MultiArray gains_msg;
  gains_msg.data.resize(29);
  if (robot == 0) {
    gains_msg.data = gains0;
    gains_pub0->publish(gains_msg);
    ROS_INFO("Experiment manager: Sent gains message, robot %d.",robot);
  } else if (robot == 1) {
    gains_msg.data = gains1;
    gains_pub1->publish(gains_msg);
    ROS_INFO("Experiment manager: Sent gains message, robot %d.",robot);
  } else {
    ROS_ERROR("Experiment manager: Unknown robot selected for gains message.");
  }
}

// Stop data logging
void StopDataLogging() {
  std_srvs::Empty srv_stop_logging;
  //ROS_INFO("Experiment manager: Calling log stop service.");
  if (client_stop_logging->call(srv_stop_logging)) {
    //ROS_INFO("Experiment manager: Stopped logging.");
  } else {
    ROS_ERROR("Experiment manager: Failed to call service start_logging.");
  }
  //ROS_INFO("Experiment manager: Finished calling log stop service.");
}

// Start data logging
void StartDataLogging() {
  wam_impedance_experiment::StartLogging srv_start_logging;
  srv_start_logging.request.task = (int)task;
  srv_start_logging.request.subjectNumber = subjectNumber;
  srv_start_logging.request.trialNumber = trialNumber;
  srv_start_logging.request.setNumber = setNumber;
  srv_start_logging.request.set_type = (int)set_type;
  srv_start_logging.request.direction = direction1;
  srv_start_logging.request.admittance = admittance;
  srv_start_logging.request.deadband = deadband;
  if (client_start_logging->call(srv_start_logging)) {
    //ROS_INFO("Experiment manager: Started logging.");
  } else {
    ROS_ERROR("Experiment manager: Failed to call service start_logging.");
  }
}

// Write logged data to file
void WriteDataFile(int userRating) {
  wam_impedance_experiment::Int srv_write_data;
  srv_write_data.request.data = userRating;
  ROS_INFO("Experiment manager: Writing data to file...");
  if (client_write_data->call(srv_write_data)) {
    ROS_INFO("Experiment manager: Done.");
  } else {
    ROS_ERROR("Experiment manager: Failed to call service write_data.");
  }
}

// Send state change to graphics node
void StateChange() {
  wam_impedance_experiment::ChangeState srv_change_state;
  srv_change_state.request.state = (int)state;
  srv_change_state.request.trial_number = trialNumber;
  srv_change_state.request.nTrials = nTrials;
  srv_change_state.request.trial_length = trialLength;
  srv_change_state.request.set_type = int(set_type);
  if (client_change_state->call(srv_change_state)) {
    //ROS_INFO("Experiment manager: State changed: %d.",(int)state);
  } else {
    ROS_ERROR("Experiment manager: Failed to call service change_state.");
  }
  *t_start = ros::Time::now();
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

// Get trial info from file
void GetTrialInfo() {
	char fname[300]; // file name plus path
	FILE *input_file;
	char c;
	int nRead;

  //TODO: fstream
  sprintf(fname,"%s/set_info/Subject%02d.txt",pathname.c_str(),subjectNumber);
  input_file = fopen(fname,"r");
  if(input_file==NULL) {
    ROS_ERROR("Experiment manager: Can't open file %s.",fname);
  }
  for (setNumber=0; setNumber<MAX_N_SETS; setNumber++) {
    // read task type for this set
    nRead = fscanf(input_file, "%d", &trials[setNumber].task); // 0 for tracking, 1 for forcemin
    if (nRead == EOF) {
      // no more sets to read
      nSets = setNumber;
      ROS_DEBUG("Experiment manager: nSets = %d",nSets);
      break;
    }
    ROS_DEBUG("Experiment manager: Reading set %d",setNumber+1);

    // practice (0), or experiment (1)
    nRead = fscanf(input_file, "%d", &trials[setNumber].set_type);

    // read number of trials in this set
    nRead = fscanf(input_file, "%d", &trials[setNumber].nTrials);

    // trial length in seconds
    nRead = fscanf(input_file, "%f", &trials[setNumber].trialLength);

    // define trials
    for (int i=0; i<trials[setNumber].nTrials; i++) {
      ROS_DEBUG("Experiment manager: Reading trial %d",i+1);

      // get impedance parameters for this trial
      nRead = fscanf(input_file, "%f", &trials[setNumber].k1[i]);
      nRead = fscanf(input_file, "%f", &trials[setNumber].b1[i]);

      // get direction multiplier for this trial
      nRead = fscanf(input_file, "%f", &trials[setNumber].direction[i]);

      c = fgetc(input_file); // there is an extra line break here for readability
      c = fgetc(input_file);
    }

  }
  fclose(input_file);

  // get set up for first trial
  setNumber = 1;
  trialNumber = 1;
  task = (taskType)trials[setNumber-1].task;
  nTrials = trials[setNumber-1].nTrials;
  trialLength = trials[setNumber-1].trialLength;
  set_type = (setType)trials[setNumber-1].set_type;
  imp1.k = trials[setNumber-1].k1[trialNumber-1];
  imp1.b = trials[setNumber-1].b1[trialNumber-1];
  direction1 = trials[setNumber-1].direction[trialNumber-1];

}


void NextTrial(void) {
  // increment set number, trial number, etc.
  // and set next state as appropriate
  if (trialNumber >= nTrials) {
    if (setNumber >= nSets) {
      trialNumber = 1;
      setNumber++; // this is just an indicator now.
      state = experimentDone;
    } else {
      // take a break after every set
      state = timedBreak;
      trialNumber = 1;
      setNumber++;
      task = (taskType)trials[setNumber-1].task;
      nTrials = trials[setNumber-1].nTrials;
      trialLength = trials[setNumber-1].trialLength;
      set_type = (setType)trials[setNumber-1].set_type;
      imp1.k = trials[setNumber-1].k1[trialNumber-1];
      imp1.b = trials[setNumber-1].b1[trialNumber-1];
      direction1 = trials[setNumber-1].direction[trialNumber-1];
    }
  } else {
    trialNumber++;
    imp1.k = trials[setNumber-1].k1[trialNumber-1];
    imp1.b = trials[setNumber-1].b1[trialNumber-1];
    direction1 = trials[setNumber-1].direction[trialNumber-1];
    state = waitingForUser;
  }
  StateChange();;
  ROS_INFO("Experiment manager: Set %d, trial %d. k=%f, b=%f, task=%d, set_type=%d, dir=%d.",setNumber,trialNumber,imp1.k,imp1.b,task,set_type,direction1);
}


void SendControlInfo() {

  wam_impedance_experiment::ControllerInfo ctrl_info_msg;
  if (robot == 0) {
    ctrl_info_msg.controllerType = (int)controller0;
    ctrl_info_msg.setpoint_null.resize(7);
    ctrl_info_msg.setpoint_null = setpoint_null_0;
    ctrl_info_pub0->publish(ctrl_info_msg);
    ROS_INFO("Experiment manager: Sent control info message, robot %d, controller %d.",robot,ctrl_info_msg.controllerType);
  } else if (robot == 1) {
    ctrl_info_msg.controllerType = (int)controller1;
    ctrl_info_msg.setpoint_null.resize(7);
    ctrl_info_msg.setpoint_null = setpoint_null_1;
    ctrl_info_pub1->publish(ctrl_info_msg);
    ROS_INFO("Experiment manager: Sent control info message, robot %d, controller %d.",robot,ctrl_info_msg.controllerType);
  } else {
    ROS_ERROR("Experiment manager: Unknown robot selected for control info message.");
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
    ROS_INFO("Experiment manager: Sent trajectory info message, robot %d, trajectory %d.",robot, traj_info_msg.trajectoryType);
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
    ROS_INFO("Experiment manager: Sent trajectory info message, robot %d, trajectory %d.",robot, traj_info_msg.trajectoryType);
  } else {
    ROS_ERROR("Experiment manager: Unknown robot selected for trajectory info message.");
  }

}

void PreviousTrial(void) {
  // decrement set number, trial number, etc.
  // and set state as appropriate
  if (trialNumber > 1) {
    // staying within the same set; just update trial information
    trialNumber--;
    imp1.k = trials[setNumber-1].k1[trialNumber-1];
    imp1.b = trials[setNumber-1].b1[trialNumber-1];
    direction1= trials[setNumber-1].direction[trialNumber-1];
    state = waitingForUser;
  } else if (setNumber > 1) {
    // changing sets
    // get set info first, so we know how many trials are in the set
    setNumber--;
    task = (taskType)trials[setNumber-1].task;
    nTrials = trials[setNumber-1].nTrials;
    trialLength = trials[setNumber-1].trialLength;
    set_type = (setType)trials[setNumber-1].set_type;
    // go to last trial of set
    trialNumber = nTrials;
    // get trial info
    imp1.k = trials[setNumber-1].k1[trialNumber-1];
    imp1.b = trials[setNumber-1].b1[trialNumber-1];
    direction1 = trials[setNumber-1].direction[trialNumber-1];
    state = setup2;
    // activate jointspace PD, for right arm and grav comp for left arm.
    robot = 0;
    controller0 = gravityCompensation;
    SendControlInfo();
    robot = 1;
    controller1 = computedTorque_Rn;
    SendControlInfo();

    StateChange();
  } else {
    // already at first set, first trial; do nothing
  }
  StateChange();
  ROS_INFO("Experiment manager: Set %d, trial %d. k=%f, b=%f, task=%d, set_type=%d, dir=%d.",setNumber,trialNumber,imp1.k,imp1.b,task,set_type,direction1);
}


void ButtonPress(void) {
  std_msgs::Empty ctrl_enable_msg;

  // advance state, as appropriate
  switch (state) {

    case setup1:
      // require debug mode to continue, so that the user can't do it themselves.
      // when starting setup2, activate trajectories and controllers.
      if (debug_mode) {
        state = setup2;

        // send ctrl enable signal
        ctrl_enable_pub->publish(ctrl_enable_msg);
        ROS_INFO("Experiment manager: Sent controller enable message.");

        // activate jointspace PD, for right arm and grav comp for left arm.
        robot = 0;
        controller0 = gravityCompensation;
        SendGains();
        SendControlInfo();
        robot = 1;
        controller1 = computedTorque_Rn;
        SendGains();
        SendControlInfo();

        StateChange();
        debug_mode = false;
      }
      break;

    case setup2:
      // nothing here, because this state is advanced automatically elsewhere
      break;

    case setup3:
      // require debug mode to continue, so that the user can't do it themselves
      if (debug_mode) {
        if (task == tracking) {
          state = setupTracking1;

          // activate jointspace PD, both arms
          robot = 0;
          controller0 = computedTorque_Rn;
          SendControlInfo();
          robot = 1;
          controller1 = computedTorque_Rn;
          SendControlInfo();

          // activate Rn setpoint trajectory and move to appropriate setpoint, both arms
          robot = 0;
          trajectory0 = setpointsRn;
          Rn_0 = Rn0_start;
          SendTrajectoryInfo();
          robot = 1;
          trajectory1 = setpointsRn;
          Rn_1 = Rn_home;
          SendTrajectoryInfo();

          StateChange();
          debug_mode = false;

        } else if (task == forcemin) {
          state = setupForcemin1;

          // activate jointspace PD, both arms
          robot = 0;
          controller0 = computedTorque_Rn;
          SendControlInfo();
          robot = 1;
          controller1 = computedTorque_Rn;
          SendControlInfo();

          // activate Rn setpoint trajectory and move to appropriate setpoint, both arms
          robot = 0;
          trajectory0 = setpointsRn;
          Rn_0 = Rn0_start;
          SendTrajectoryInfo();
          robot = 1;
          trajectory1 = setpointsRn;
          Rn_1 = Rn1_start;
          SendTrajectoryInfo();

          StateChange();
          debug_mode = false;

        } else {

          ROS_ERROR("Experiment manager: Unknown task type.");

        }

      }
      break;

    case setupTracking1:
    case setupTracking2:
    case setupForcemin1:
    case setupForcemin2:
      // nothing here, because this state is advanced automatically elsewhere
      break;

    case setupForcemin3:
      // require debug mode to continue, so that the user can't do it themselves
      if (debug_mode) {
        state = setupForcemin4;

        // activate impedance controller, both arms
        robot = 0;
        controller0 = impedance_SE3_null_fric;
        SendControlInfo();
        robot = 1;
        controller1 = impedance_SE3_null_fric;
        SendControlInfo();

        StateChange();
        debug_mode = false;
      }
      break;

    case setupForcemin4:
      // nothing here, because this state is advanced automatically elsewhere
      break;

    case graphicsCalibration1:
      if (debug_mode) {
        state = graphicsCalibration2;
        robot = 0;
        trajectory0 = setpointsRn;
        Rn_0 = Rn0_right;
        SendTrajectoryInfo();
        StateChange();
        debug_mode = false;
      }
      break;

    case graphicsCalibration2:
    case graphicsCalibration4:
      // nothing here, because this state is advanced automatically elsewhere
      break;

    case graphicsCalibration3:
      if (debug_mode) {
        state = graphicsCalibration4;
        robot = 0;
        trajectory0 = setpointsRn;
        Rn_0 = Rn0_start;
        SendTrajectoryInfo();
        StateChange();
        debug_mode = false;
      }
      break;

    case instructions:
      // advance state
      state = waitingForUser;
      StateChange();
      break;

    case waitingForUser:
      // send appropriate controller gains. note that in the cisst controller, receiving new gains
      // does not activate a controller transition period. to force a controller transition, switch
      // to gravity compensation, send gains, and switch back to impedance.
      robot = 0;
      controller0 = gravityCompensation;
      SendControlInfo();
      gains0[15] = imp1.k;
      gains0[21] = imp1.b;
      SendGains();
      controller0 = impedance_SE3_null_fric;
      SendControlInfo();
      if (task == forcemin) {
        robot = 1;
        controller1 = gravityCompensation;
        SendControlInfo();
        gains1[15] = imp2.k;
        gains1[21] = imp2.b;
        SendGains();
        controller1 = impedance_SE3_null_fric;
        SendControlInfo();
      }
      state = startingTrial1;
      limitR = 4.0;
      limitL = 4.0;
      phase = -2.1;
      StateChange();
      break;

    case startingTrial1:
    case startingTrial2:
    case runningTrial:
      // nothing here, because this state is advanced automatically elsewhere
      break;

    case rating:
      // nothing happens here; state is advanced when number key is pressed
      break;

    case writingDataFile:
    case resetWam1:
    case resetWam2:
    case resetWam3:
      // nothing here, because this state is advanced automatically elsewhere
      break;

    case timedBreak:
      // require debug mode to continue, so that the user can't do it themselves
      if (debug_mode) {
        // before certain sets (3, 5, 6, 8), go to cleanup mode so the experimenter can restart the cisst node.
        // before certain other sets (4, 7), go to instructions because the task isn't changing so there's no setup.
        // before set 2, there will be setup to do.
        if ((setNumber == 3) || (setNumber == 5) || (setNumber == 6) || (setNumber == 8)) {

          state = cleanup1;

          robot = 0;
          controller0 = computedTorque_Rn;
          SendControlInfo();

          robot = 0;
          controller0 = gravityCompensation;
          SendControlInfo();

          StateChange();
          debug_mode = false;

        } else if ((setNumber == 4) || (setNumber == 7)) {

          state = instructions;
          StateChange();
          debug_mode = false;

        } else {

          if (task == tracking) {
            state = setup2;

            // activate jointspace PD for right arm and gravity compensation for left arm
            robot = 0;
            controller0 = gravityCompensation;
            SendControlInfo();
            robot = 1;
            controller1 = computedTorque_Rn;
            SendControlInfo();

            StateChange();
            debug_mode = false;

          } else if (task == forcemin) {
            state = setup2;

            // activate jointspace PD for right arm and gravity compensation for left arm
            robot = 0;
            controller0 = gravityCompensation;
            SendControlInfo();
            robot = 1;
            controller1 = computedTorque_Rn;
            SendControlInfo();

            StateChange();
            debug_mode = false;

          } else {

            ROS_ERROR("Experiment manager: Unknown task type.");

          }

        }
      }
      break;

    case experimentDone:
      // require debug mode to continue, so that the user can't do it themselves
      if (debug_mode) {
        state = cleanup1;

        robot = 0;
        controller0 = computedTorque_Rn;
        SendControlInfo();

        if (task == forcemin) {
          robot = 0;
          controller0 = gravityCompensation;
          SendControlInfo();
        }

        StateChange();
        debug_mode = false;
      }
      break;

    case cleanup1:
      // do nothing here, because the state is automatically advanced elsewhere.

    case cleanup2:
      // require debug mode to continue, so that the user can't do it themselves
      if (debug_mode) {
        state = cleanup3;

        robot = 0;
        controller0 = computedTorque_Rn;
        SendControlInfo();
        robot = 1;
        controller1 = computedTorque_Rn;
        SendControlInfo();
        robot = 0;
        trajectory0 = setpointsRn;
        Rn_0 = Rn_home;
        SendTrajectoryInfo();
        robot = 1;
        trajectory1 = setpointsRn;
        Rn_1 = Rn_home;
        SendTrajectoryInfo();

        StateChange();
        debug_mode = false;
      }
      break;

    case cleanup3:
      // nothing here, because this state is advanced automatically elsewhere
      break;

    case cleanup4:
      // presumably the experimenter has restarted the cisst node and is ready to move on.
      if (debug_mode && (setNumber <= nSets)) {
        state = setup1;
        StateChange();
        debug_mode = false;
      }
      break;


    default:
      // this should never happen...
      ROS_WARN("Experiment manager: Unknown state.");
      break;
  }
}


/*
bool handleButtonPress(std_srvs::Empty::Request  &req,
                       std_srvs::Empty::Response &res )
{
  ButtonPress();
  return true;
}
*/

void trajectoryCallback(const wam_impedance_experiment::TrajectoryInfo::ConstPtr& msg)
{
  // get current commanded position of robot
  std::string name = msg->name;
  if ((strcmp(name.c_str(),"robot0") == 0) || (strcmp(name.c_str(),"left") == 0)) {
    for (int i=0;i<7;i++) {
      Rn0_current[i] = msg->setpointRn[i];
    }
  } else if ((strcmp(name.c_str(),"robot1") == 0) || (strcmp(name.c_str(),"right") == 0)) {
    for (int i=0;i<7;i++) {
      Rn1_current[i] = msg->setpointRn[i];
    }
  } else {
    ROS_ERROR("Experiment manager: trajectoryCallback: Unknown robot name.");
  }
}

void keyPressCallback(const std_msgs::Int32::ConstPtr& msg)
{
  int key = msg->data;
  ROS_INFO("Experiment manager: Received key press %d.",key);

  std_msgs::Empty ctrl_enable_msg;
  
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

    case KEY_W:
      // start or stop data logging
      if (logging) {
        StopDataLogging();
      } else {
        StartDataLogging();
      }
      logging = !logging;
      break;

    case KEY_1:
    case KEY_2:
    case KEY_3:
    case KEY_4:
    case KEY_5:
      // select a rating during rating mode.
      if (state == rating) {
        // save rating - rating is equal to key number minus 48
        user_rating = key-48;
        // advance state
        state = writingDataFile;
        StateChange();
      }
      break;

    case KEY_SPC:
      if (debug_mode) {
        ButtonPress();
      }
      break;

    case KEY_ENTER:
      ButtonPress();
      break;

    case KEY_TILDE:
      // toggle debug mode
      debug_mode = !debug_mode;
      break;

    case KEY_N:
    case KEY_SHIFT_N:
      if (debug_mode) {
        if ((state == rating) || (state == runningTrial)) {
          if (task == tracking) {

            if (state == runningTrial) {
              // need to disable projectiles.
              ToggleProjectiles();
            }

            state = resetWam1;

            // activate Rn setpoint trajectory and move to appropriate setpoint, both arms
            robot = 0;
            trajectory0 = setpointsRn;
            Rn_0 = Rn0_start;
            SendTrajectoryInfo();
            robot = 1;
            trajectory1 = setpointsRn;
            Rn_1 = Rn_home;
            SendTrajectoryInfo();

            StateChange();

          } else if (task == forcemin) {

            state = resetWam1;

            // activate Rn setpoint trajectory and move to appropriate setpoint, both arms
            robot = 0;
            trajectory0 = setpointsRn;
            Rn_0 = Rn0_start;
            SendTrajectoryInfo();
            robot = 1;
            trajectory1 = setpointsRn;
            Rn_1 = Rn1_start;
            SendTrajectoryInfo();

            StateChange();

          } else {

            ROS_ERROR("Experiment manager: Unknown task type.");

          }

          // skip next to trial
          if (key == KEY_SHIFT_N) {
            trialNumber = nTrials;
          }
          NextTrial();

        } else if ((state == instructions) || (state == waitingForUser)) {

          //skip to next trial
          if (key == KEY_SHIFT_N) {
            trialNumber = nTrials;
          }
          NextTrial();

        }
        debug_mode = false;
      }
      break;

    case KEY_P:
      if (debug_mode) {
        if ((state == experimentDone) || (state == instructions) || (state == waitingForUser) || (state == timedBreak)) {
          // skip back to previous trial
          PreviousTrial();
        }
        debug_mode = false;
      }
      break;

    case KEY_SHIFT_P:
      if (debug_mode) {
        if ((state == experimentDone) || (state == instructions) || (state == waitingForUser) || (state == timedBreak)) {
          if (trialNumber == 1) {
            if (setNumber > 1) {
              // go to last trial of previous set
              PreviousTrial();
            }
          } else {
            // go to first trial of current set
            trialNumber = 2;
            PreviousTrial();
          }
        }
        debug_mode = false;
      }
      break;

    case KEY_BANG:
      // put both robots into gravity compensation. this is for the purpose of short
      // circuiting the rest of the experiment if needed.
      if (debug_mode) {
        if ((task == tracking) && (state == runningTrial)) {
          // turn off projectiles.
          ToggleProjectiles();
        }
        robot = 0;
        controller0 = gravityCompensation;
        SendControlInfo();
        robot = 1;
        controller1 = gravityCompensation;
        SendControlInfo();

        state = cleanup4;
        StateChange();

        ROS_WARN("Experiment manager: Put both robots in gravity compensation mode. Please clean up.");
        debug_mode = !debug_mode;
      }
      break;

    default:
      ROS_INFO("Experiment manager: Pressing %d doesn't do anything.",key);
      break;

  }

}


int main(int argc, char **argv) {

  // initialize gains
  imp1.k = 200; // stiffness of virtual prosthesis
  imp2.k = 100; // stiffness of moving object in LI task
  imp1.b = 25; // viscosity of virtual prosthesis
  imp2.b = 1; // viscosity of moving object in LI task


  // set up ROS stuff
  ros::init(argc, argv, "experiment_manager");
  n = new ros::NodeHandle;
  ros::Rate loop_rate(100);
  ros::Time t;
  ros::Time told;
  ros::Duration deltat;

  n->setParam("/admittance",admittance);
  n->setParam("/deadband",deadband);

  ctrl_enable_pub = new ros::Publisher;
  *ctrl_enable_pub = n->advertise<std_msgs::Empty>("ctrl_enable",100);

  toggle_projectiles_pub = new ros::Publisher;
  *toggle_projectiles_pub = n->advertise<std_msgs::Empty>("toggle_projectiles",100);

  toggle_verbose_logging_pub = new ros::Publisher;
  *toggle_verbose_logging_pub = n->advertise<std_msgs::Empty>("toggle_verbose_logging",100);

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
  gains0[26] = 0.4;     gains0[27] = 0.1;
  // friction gain
  gains0[28] = 0.5;

  gains_pub1 = new ros::Publisher;
  *gains_pub1 = n->advertise<std_msgs::Float64MultiArray>("gains_1",100);
  gains1.resize(29);
  // P gains Rn          // D gains Rn
  gains1[0] = 800;       gains1[7] = 12.0;
  gains1[1] = 600;       gains1[8] = 6.0;
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
  gains1[26] = 0.8;      gains1[27] = 0.1;
  // friction gain
  gains1[28] = 0.5;

  traj_info_pub0 = new ros::Publisher;
  *traj_info_pub0 = n->advertise<wam_impedance_experiment::TrajectoryInfo>("traj_info_0",100);

  traj_info_pub1 = new ros::Publisher;
  *traj_info_pub1 = n->advertise<wam_impedance_experiment::TrajectoryInfo>("traj_info_1",100);

  client_start_logging = new ros::ServiceClient;
  *client_start_logging = n->serviceClient<wam_impedance_experiment::StartLogging>("start_logging");
  wam_impedance_experiment::StartLogging srv_start_logging;

  client_stop_logging = new ros::ServiceClient;
  *client_stop_logging = n->serviceClient<std_srvs::Empty>("stop_logging");
  std_srvs::Empty srv_stop_logging;

  client_write_data = new ros::ServiceClient;
  *client_write_data = n->serviceClient<wam_impedance_experiment::Int>("write_data");

  client_zero_force = new ros::ServiceClient;
  *client_zero_force = n->serviceClient<wam_impedance_experiment::Int>("zero_force");

  // subscriber for receiving key presses
  ros::Subscriber sub_key_press = n->subscribe<std_msgs::Int32>("key_press", 1000, keyPressCallback);

  // subscriber for receiving quit command
  ros::Subscriber sub_quit = n->subscribe<std_msgs::Empty>("quit", 1000, Quit);

  // publisher for communicating with graphics node
  ros::Publisher graphics_info_pub = n->advertise<wam_impedance_experiment::GraphicsInfo>("graphics_info", 1000);
  wam_impedance_experiment::GraphicsInfo msg_graphics_info;

  // services for communicating with graphics node
  ros::ServiceClient client_set_graphics_mode = n->serviceClient<wam_impedance_experiment::Bool>("set_graphics_mode");
  client_change_state = new ros::ServiceClient;
  *client_change_state = n->serviceClient<wam_impedance_experiment::ChangeState>("change_state");
  wam_impedance_experiment::Bool srv_set_graphics_mode;

  t_start = new ros::Time;
  *t_start = ros::Time::now();

  // subscriber for receiving trajectory. lets experiment manager know when cisst
  // trajectory has reached a commanded setpoint.
  ros::Subscriber sub_trajectory = n->subscribe<wam_impedance_experiment::TrajectoryInfo>("trajectory", 1000, trajectoryCallback);

  //std_srvs::Empty srv_start_movement;

  // get subject number
  if (auto_run) {
    subjectNumber = 0;
  } else if (n->getParam("/subjectNumber", subjectNumber)) {
    if (subjectNumber == 0) {
      ROS_WARN("Experiment manager: Using default subject number 0. To specify a different subject number, use \"n:=##\" with the roslaunch command.");
    } else {
      ROS_INFO("Experiment manager: Running experiment for subject number %d.",subjectNumber);
    }
  } else {
    ROS_WARN("Experiment manager: Unable to retrieve subject number from parameter server. Please provide subject number.");
    std::cout << "Subject number? ";
    std::cin >> subjectNumber;
  }

  // get set/trial info
  pathname = ros::package::getPath("wam_impedance_experiment");
  GetTrialInfo();
  ROS_INFO("Experiment manager: Trial info loaded.");

  // initialize srv_set_graphics_mode
  srv_set_graphics_mode.request.data = debug_mode;

  // home position (initial robot pose)
  Rn_home[1] = -cmnPI_2;
  Rn_home[3] =  cmnPI;

  // nullspace setpoint
  setpoint_null_0[2] = 0.8;
  setpoint_null_1[2] = -1.5;

  // start position for trials
  Rn0_start[0] = 0.85200066108372685;
  Rn0_start[1] = -0.71117053781398032;
  Rn0_start[2] = 1.4398269787074245;
  Rn0_start[3] = 1.4624115427739803;
  Rn0_start[4] = -0.70173978609938803;
  Rn0_start[5] = 0.83672971930812079;
  Rn0_start[6] = -0.22644661266545243;
  Rn1_start[0] = -0.84352779773070785;
  Rn1_start[1] = -0.74601279380506202;
  Rn1_start[2] = -1.3807088353951313;
  Rn1_start[3] = 1.4624115427739641;
  Rn1_start[4] = 0.61414993604158652;
  Rn1_start[5] = 0.79927303718499287;
  Rn1_start[6] = 0.29283240889511913;
  /*
  Rn0_start[0] = 0.839125;
  Rn0_start[1] = -0.760694;
  Rn0_start[2] = 1.356141;
  Rn0_start[3] = 1.4624115;
  Rn0_start[4] = -0.574992;
  Rn0_start[5] = 0.784711;
  Rn0_start[6] = -1.893884;
  Rn1_start[0] = -0.839125;
  Rn1_start[1] = -0.760694;
  Rn1_start[2] = -1.356141;
  Rn1_start[3] = 1.4624115;
  Rn1_start[4] = 0.574992;
  Rn1_start[5] = 0.784711;
  Rn1_start[6] = 1.893884;
  */

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

  // loop: define and publish data

  while (ros::ok()) {

    // get timestamp and period
    told = t;
    t = ros::Time::now();
    deltat = t - told;

    // do state-appropriate things!
    double distance0 = 0.0;
    double distance1 = 0.0;
    switch (state) {

      case setup1:
        ROS_DEBUG("State: setup1");
        // do nothing here; we're waiting for the experimenter to advance the state
        break;

      case setup2:
        ROS_DEBUG("State: setup2");
        // wait specified amount of time for controller transition, then advance state.
        if ((t - *t_start).toSec() >= controllerTransitionTime) {
          state = setup3;
          StateChange();
        }
        break;

      case setup3:
        ROS_DEBUG("State: setup3");
        // do nothing here; we're waiting for the experimenter to advance the state
        break;

      case setupTracking1:
        ROS_DEBUG("State: setupTracking1");
        // wait specified amount of time for controller transition,
        // and check for wam positions.
        // then advance state.

        // first calculate distance between commanded setpoints and current setpoints by
        // taking the length of the difference vector.
        // note that Rn_0 and Rn_1 are the most recently commanded setpoints from the
        // experiment manager, while Rn0_current and Rn1_current are the most recently
        // commanded setpoints from the cisst trajectory.
        distance0 = 0.0;
        distance1 = 0.0;
        for (int i=0;i<7;i++) {
          distance0 += pow((Rn_0[i]-Rn0_current[i]),2);
          distance1 += pow((Rn_1[i]-Rn1_current[i]),2);
        }
        distance0 = pow(distance0,0.5);
        distance1 = pow(distance1,0.5);

        // then check that the distance is close and make sure controller transition
        // time has passed.
        if ((t - *t_start).toSec() >= controllerTransitionTime) {
          if ((distance0 < 0.01) && (distance1 < 0.01)) {
            state = setupTracking2;
            robot = 0;
            controller0 = impedance_SE3_null_fric;
            SendControlInfo();
            StateChange();
          }
        }
        break;

      case setupTracking2:
        ROS_DEBUG("State: setupTracking2");
        if ((t - *t_start).toSec() >= controllerTransitionTime) {
          if (setNumber == 1) {
            state = graphicsCalibration1;
          } else {
            state = instructions;
          }
          StateChange();
        }
        break;

      case setupForcemin1:
        ROS_DEBUG("State: setupForcemin1");
        // wait specified amount of time for controller transition,
        // and check for wam positions.
        // then advance state.

        // first calculate distance between commanded setpoints and current setpoints by
        // taking the length of the difference vector.
        // note that Rn_0 and Rn_1 are the most recently commanded setpoints from the
        // experiment manager, while Rn0_current and Rn1_current are the most recently
        // commanded setpoints from the cisst trajectory.
        distance0 = 0.0;
        distance1 = 0.0;
        for (int i=0;i<7;i++) {
          distance0 += pow((Rn_0[i]-Rn0_current[i]),2);
          distance1 += pow((Rn_1[i]-Rn1_current[i]),2);
        }
        distance0 = pow(distance0,0.5);
        distance1 = pow(distance1,0.5);

        // then check that the distance is close and make sure controller transition
        // time has passed.
        if ((t - *t_start).toSec() >= controllerTransitionTime) {
          if ((distance0 < 0.01) && (distance1 < 0.01)) {
            state = setupForcemin2;
            // zero environment force sensor. note this is blocking (on purpose).
            ZeroForceSensor(2);
            // put left wam into grav comp (for this one, shouldn't need to wait for transition)
            robot = 0;
            controller0 = gravityCompensation;
            SendControlInfo();
            StateChange();
          }
        }
        break;

      case setupForcemin2:
        ROS_DEBUG("State: setupForcemin2");
        // wait specified amount of time for controller transition, then advance state.
        if ((t - *t_start).toSec() >= controllerTransitionTime) {
          state = setupForcemin3;
          StateChange();
        }

        break;

      case setupForcemin3:
        ROS_DEBUG("State: setupForcemin3");
        // do nothing here; we're waiting for the experimenter to advance the state
        break;

      case setupForcemin4:
        ROS_DEBUG("State: setupForcemin4");
        // wait specified amount of time for controller transition, then advance state.
        if ((t - *t_start).toSec() >= controllerTransitionTime) {
          if (setNumber == 1) {
            state = graphicsCalibration1;
          } else {
            state = instructions;
          }
          StateChange();
        }
        break;

      case graphicsCalibration1:
        ROS_DEBUG("State: graphicsCalibration1");
        // do nothing here; we're waiting for the experimenter to advance the state
        // or, in auto_run mode, advance state automatically.
        if (auto_run) {
          ButtonPress();
        }
        break;

      case graphicsCalibration2:
        ROS_DEBUG("State: graphicsCalibration2");

        // wait for trajectory transition to complete. then advance state automatically.

        // first calculate distance between commanded setpoints and current setpoints by
        // taking the length of the difference vector.
        // note that Rn_0 and Rn_1 are the most recently commanded setpoints from the
        // experiment manager, while Rn0_current and Rn1_current are the most recently
        // commanded setpoints from the cisst trajectory.
        distance0 = 0.0;
        distance1 = 0.0;
        for (int i=0;i<7;i++) {
          distance0 += pow((Rn_0[i]-Rn0_current[i]),2);
          distance1 += pow((Rn_1[i]-Rn1_current[i]),2);
        }
        distance0 = pow(distance0,0.5);
        distance1 = pow(distance1,0.5);

        if ((distance0 < 0.01) && (distance1 < 0.01)) {
          state = graphicsCalibration3;
          StateChange();
        }
        break;

      case graphicsCalibration3:
        ROS_DEBUG("State: graphicsCalibration3");
        // do nothing here; we're waiting for the experimenter to advance the state
        // or, in auto_run mode, advance state automatically.
        if (auto_run) {
          ButtonPress();
        }
        break;

      case graphicsCalibration4:
        ROS_DEBUG("State: graphicsCalibration4");

        // wait for trajectory transition to complete. then advance state automatically.

        // first calculate distance between commanded setpoints and current setpoints by
        // taking the length of the difference vector.
        // note that Rn_0 and Rn_1 are the most recently commanded setpoints from the
        // experiment manager, while Rn0_current and Rn1_current are the most recently
        // commanded setpoints from the cisst trajectory.
        distance0 = 0.0;
        distance1 = 0.0;
        for (int i=0;i<7;i++) {
          distance0 += pow((Rn_0[i]-Rn0_current[i]),2);
          distance1 += pow((Rn_1[i]-Rn1_current[i]),2);
        }
        distance0 = pow(distance0,0.5);
        distance1 = pow(distance1,0.5);

        if ((distance0 < 0.01) && (distance1 < 0.01)) {
          state = instructions;
          StateChange();
        }
        break;

      case instructions:
        ROS_DEBUG("State: instructions");
        // do nothing here; we're waiting for the user to press the button.
        // or, in auto_run mode, advance state automatically.
        if (auto_run) {
          ButtonPress();
        }
        break;

      case waitingForUser:
        ROS_DEBUG("State: waitingForUser");
        // do nothing here; we're waiting for the user to press the button.
        // or, in auto_run mode, advance state automatically.
        if (auto_run) {
          ButtonPress();
        }
        break;

      case startingTrial1:
        ROS_DEBUG("State: startingTrial1");
        // wait specified amount of time for controller transition, then advance state.
        if ((t - *t_start).toSec() >= controllerTransitionTime) {
          if (task == forcemin) {
            ZeroForceSensor(2);
          }
          state = startingTrial2;
          robot = 0;
          trajectory0 = userInput;
          SendTrajectoryInfo();
          StateChange();
        }
        break;

      case startingTrial2:
        ROS_DEBUG("State: startingTrial2");
        // keep track of time, check for start of trial
        if ((t - *t_start).toSec() >= trialStartLength) {
          state = runningTrial;
          StartDataLogging();
          if (task == forcemin) {
            robot = 1;
            trajectory1 = sinusoid;
            SendTrajectoryInfo();
          } else if (task == tracking) {
            ToggleProjectiles();
          }
          StateChange();
        } else {
          // for forcemin task, just wait. for tracking task, move the sine wave.
          if (task == tracking) {
            phase += deltat.toSec();
            /*
            if (phase > 0.0) {
              phase = 0.0;
            }
            */
            if (phase <= 4.0) {
              limitL = -1.0*phase;
            } else {
              limitL = -4.0;
            }
            limitR = 4.0;
          }
        }
        break;

      case runningTrial:
        ROS_DEBUG("State: runningTrial");
        // keep track of time, check for end of trial
        if ((t - *t_start).toSec() >= trialLength) {
          if (task == tracking) {
            ToggleProjectiles();
          }
          robot = 0;
          trajectory0 = stationary;
          SendTrajectoryInfo();
          robot = 1;
          trajectory1 = stationary;
          SendTrajectoryInfo();
          StopDataLogging();
          limitL = 4.0;
          phase = -2.1;
          state = rating;
          StateChange();
        } else {
          if (task == tracking) {
           phase += deltat.toSec();
            if (phase <= 4.0) {
              limitL = -1.0*phase;
            } else {
              limitL = -4.0;
            }
          }
        }
        break;

      case rating:
        ROS_DEBUG("State: rating");
        // do nothing here; we're waiting for the user to enter a number.
        // or, in auto_run mode, advance state automatically.
        if (auto_run) {
          user_rating = 3;
          state = writingDataFile;
          StateChange();
        }

        break;

      case writingDataFile:
        ROS_DEBUG("State: writingDataFile");
        // write the data file (blocking) and then advance the state automatically.
        WriteDataFile(user_rating);

        if (task == tracking) {

          state = resetWam1;

          // activate Rn setpoint trajectory and move to appropriate setpoint, both arms
          robot = 0;
          trajectory0 = setpointsRn;
          Rn_0 = Rn0_start;
          SendTrajectoryInfo();
          robot = 1;
          trajectory1 = setpointsRn;
          Rn_1 = Rn_home;
          SendTrajectoryInfo();

          StateChange();

        } else if (task == forcemin) {

          state = resetWam1;

          // activate Rn setpoint trajectory and move to appropriate setpoint, both arms
          robot = 0;
          trajectory0 = setpointsRn;
          Rn_0 = Rn0_start;
          SendTrajectoryInfo();
          robot = 1;
          trajectory1 = setpointsRn;
          Rn_1 = Rn1_start;
          SendTrajectoryInfo();

          StateChange();

        } else {

          ROS_ERROR("Experiment manager: Unknown task type.");

        }
        break;
        
      case resetWam1:
        ROS_DEBUG("State: resetWam1");
        // wait for trajectory transition to complete. then advance state automatically.

        // first calculate distance between commanded setpoints and current setpoints by
        // taking the length of the difference vector.
        // note that Rn_0 and Rn_1 are the most recently commanded setpoints from the
        // experiment manager, while Rn0_current and Rn1_current are the most recently
        // commanded setpoints from the cisst trajectory.
        distance0 = 0.0;
        distance1 = 0.0;
        for (int i=0;i<7;i++) {
          distance0 += pow((Rn_0[i]-Rn0_current[i]),2);
          distance1 += pow((Rn_1[i]-Rn1_current[i]),2);
        }
        distance0 = pow(distance0,0.5);
        distance1 = pow(distance1,0.5);

        if ((distance0 < 0.01) && (distance1 < 0.01)) {

          state = resetWam2;

          // activate jointspace PD, both arms
          robot = 0;
          controller0 = computedTorque_Rn;
          SendControlInfo();
          robot = 1;
          controller1 = computedTorque_Rn;
          SendControlInfo();

          StateChange();
        }

        break;
        
      case resetWam2:
        ROS_DEBUG("State: resetWam2");
        // wait for controller transition. then advance state automatically.

        if ((t - *t_start).toSec() >= controllerTransitionTime) {
          if (task == tracking) {

            state = resetWam3;

            // activate impedance control, left wam
            robot = 0;
            controller0 = impedance_SE3_null_fric;
            SendControlInfo();

            StateChange();

          } else if (task == forcemin) {

            state = resetWam3;

            // activate impedance control, both wams
            robot = 0;
            controller0 = impedance_SE3_null_fric;
            SendControlInfo();
            robot = 1;
            controller1 = impedance_SE3_null_fric;
            SendControlInfo();

            StateChange();

          } else {
            ROS_ERROR("Experiment manager: Unknown task type.");
          }
        }

        break;

      case resetWam3:
        ROS_DEBUG("State: resetWam3");
        // wait for controller transition to repeat, then proceed automatically.
        if ((t - *t_start).toSec() >= controllerTransitionTime) {
          NextTrial();
        }
        break;

      case timedBreak:
        ROS_DEBUG("State: timedBreak");
        // do nothing here, because we are waiting for the experimenter to advance the state
        break;

      case experimentDone:
        ROS_DEBUG("State: experimentDone");
        // do nothing here, because we are waiting for the experimenter to advance the state
        break;

      case cleanup1:
        ROS_DEBUG("State: cleanup1");
        // wait specified amount of time for controller transition, then advance state.
        if ((t - *t_start).toSec() >= controllerTransitionTime) {
          state = cleanup2;
          StateChange();
        }
        break;

      case cleanup2:
        ROS_DEBUG("State: cleanup2");
        // do nothing here, because we are waiting for the experimenter to advance the state
        break;

      case cleanup3:
        ROS_DEBUG("State: cleanup3");
        // wait specified amount of time for controller transition, and check for wam positions.
        // then advance state.

        // first calculate distance between commanded setpoints and current setpoints by
        // taking the length of the difference vector.
        // note that Rn_0 and Rn_1 are the most recently commanded setpoints from the
        // experiment manager, while Rn0_current and Rn1_current are the most recently
        // commanded setpoints from the cisst trajectory.
        distance0 = 0.0;
        distance1 = 0.0;
        for (int i=0;i<7;i++) {
          distance0 += pow((Rn_0[i]-Rn0_current[i]),2);
          distance1 += pow((Rn_1[i]-Rn1_current[i]),2);
        }
        distance0 = pow(distance0,0.5);
        distance1 = pow(distance1,0.5);

        // then check that the distance is close and make sure controller transition
        // time has passed.
        if ((t - *t_start).toSec() >= (controllerTransitionTime + 1.0)) {
          if ((distance0 < 0.01) && (distance1 < 0.01)) {
            state = cleanup4;

            robot = 0;
            controller0 = gravityCompensation;
            SendControlInfo();
            robot = 1;
            controller1 = gravityCompensation;
            SendControlInfo();

            StateChange();
          }
        }
        break;

      case cleanup4:
        ROS_DEBUG("State: cleanup4");
        // do nothing here. this is the end.
        break;

      default:
        // this should never happen...
        ROS_WARN("Experiment manager: Unknown state.");
    }

    // send graphics info message, if applicable
    msg_graphics_info.t = t;
    msg_graphics_info.task = (int)task;
    msg_graphics_info.deltat = deltat;
    msg_graphics_info.direction = direction1;
    msg_graphics_info.limitL = limitL;
    msg_graphics_info.limitR = limitR;
    msg_graphics_info.phase = phase;
    graphics_info_pub.publish(msg_graphics_info);

    if (srv_set_graphics_mode.request.data != debug_mode) {
      // change graphics for debug mode
      srv_set_graphics_mode.request.data = debug_mode;
      if (client_set_graphics_mode.call(srv_set_graphics_mode)) {
        ROS_INFO("Experiment manager: Graphics mode set.");
      } else {
        ROS_ERROR("Experiment manager: Failed to call service set_graphics_mode.");
      }
    }

    
    ros::spinOnce();
    loop_rate.sleep();

  }

  delete t_start;
  delete ctrl_enable_pub;
  delete toggle_projectiles_pub;
  delete toggle_verbose_logging_pub;
  delete ctrl_info_pub0;
  delete ctrl_info_pub1;
  delete traj_info_pub0;
  delete traj_info_pub1;
  delete client_start_logging;
  delete client_stop_logging;
  delete client_write_data;
  delete client_zero_force;
  delete client_change_state;
  delete n;


  return 0;
}
