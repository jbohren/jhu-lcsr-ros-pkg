#include <stdio.h>
#include <fstream>
#include "ros/ros.h"
#include "simulation_impedance_experiment/keyboard.h"
#include "simulation_impedance_experiment/GraphicsInfo.h"
#include "simulation_impedance_experiment/SetGraphicsMode.h"
#include "simulation_impedance_experiment/ChangeState.h"
#include "simulation_impedance_experiment/TaskType.h"
#include "simulation_impedance_experiment/SetType.h"
#include "simulation_impedance_experiment/States.h"
#include "simulation_impedance_experiment/IntegerRequest.h"
#include "simulation_impedance_experiment/StartSimulation.h"
#include "simulation_impedance_experiment/Gains.h"
#include "std_srvs/Empty.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Int32.h"
#include "simulation_impedance_experiment/SetHapticFeedback.h"


int key;
bool debug_mode = false;
bool reset = false;
bool start = false;

// flags for feedback types
bool feedback_change = true;
bool vibrotactile = false;
bool skin_stretch = false;
bool visual = false;
bool change_state = false;

// for tuning PD gains (development only)
bool tunable_PD = false; // set to true for development and tuning gains, false for experiment
bool change_PD_gains = false; // flag for changed gains
int gain_key = 0;

// flags for set type
setType set_type = training;
bool variable = false; // variable vs. constant impedance
taskType task = forcemin;

// structure to hold impedance info
typedef struct {
	double k;
	double b;
	double m;
} impedance;

// gains for user and moving object
impedance imp1;
impedance imp2;

// direction multiplier for x_d2
double direction = 0.0;


const int MAX_N_SETS = 50;
const int MAX_N_TRIALS = 100;

// structure for storing trial info
typedef struct {
  float k1[MAX_N_TRIALS];
  float b1[MAX_N_TRIALS];
  float m1[MAX_N_TRIALS];
  float k2[MAX_N_TRIALS];
  float b2[MAX_N_TRIALS];
  float m2[MAX_N_TRIALS];
  float direction[MAX_N_TRIALS];
  int nTrials;        // number of trials in this set
  int task;           // task type: forcemin or tracking
  int variable;       // flag for variable impedance
  int set_type;       // practice or experiment
  int visual;         // flag for visual feedback
  int vibrotactile;   // flag for vibrotactile feedback
  int skin_stretch;   // flag for skin stretch feedback
} trialInfo;

trialInfo trials[MAX_N_SETS];

unsigned int subjectNumber = 0;
int nSets = 0;
int nTrials = 0;
int setNumber = 1;
int trialNumber = 1;
const double trialLengthPE = 15.0; // trial length for practice and experiment sets
const double trialLengthT = 60.0; // trial length for feedback training sets
double trialLength = trialLengthT;

// experiment states
stateT state = instructions;


// Get trial info from file
void GetTrialInfo(int subjectNumber) {
	char fname[100]; // file name plus path
	FILE *input_file;
	char c;
	int nRead;

  //TODO: fix this so it can run from directories other than the package directory...
  //TODO: fstream
  sprintf(fname,"set_info/Subject%02d.txt",subjectNumber);
  input_file = fopen(fname,"r");
  if(input_file==NULL) {
    ROS_ERROR("Can't open file %s.",fname);
  }
  for (setNumber=0; setNumber<MAX_N_SETS; setNumber++) {
    // read task type for this set
    nRead = fscanf(input_file, "%d", &trials[setNumber].task); // 0 for tracking, 1 for forcemin
    if (nRead == EOF) {
      // no more sets to read
      nSets = setNumber;
      setNumber = 1;
      break;
    }
    ROS_DEBUG("Reading set %d",setNumber+1);

    // read number of trials in this set
    nRead = fscanf(input_file, "%d", &trials[setNumber].nTrials);

    // variable or constant impedance?
    nRead = fscanf(input_file, "%d", &trials[setNumber].variable);

    // training (2), practice (0), or experiment (1)?
    nRead = fscanf(input_file, "%d", &trials[setNumber].set_type);

    // feedback types
    nRead = fscanf(input_file, "%d", &trials[setNumber].visual);
    nRead = fscanf(input_file, "%d", &trials[setNumber].vibrotactile);
    nRead = fscanf(input_file, "%d", &trials[setNumber].skin_stretch);

    // define trials
    for (int i=0; i<trials[setNumber].nTrials; i++) {
      // get impedance parameters for this trial
      ROS_DEBUG("Reading trial %d",i+1);

      nRead = fscanf(input_file, "%f", &trials[setNumber].k2[i]);
      nRead = fscanf(input_file, "%f", &trials[setNumber].b2[i]);
      nRead = fscanf(input_file, "%f", &trials[setNumber].m2[i]);
      nRead = fscanf(input_file, "%f", &trials[setNumber].k1[i]);
      nRead = fscanf(input_file, "%f", &trials[setNumber].b1[i]);
      nRead = fscanf(input_file, "%f", &trials[setNumber].m1[i]);

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
  variable = trials[setNumber-1].variable;
  set_type = (setType)trials[setNumber-1].set_type;
  if (set_type == training) {
    trialLength = trialLengthT;
  } else {
    trialLength = trialLengthPE;
  }
  visual = (bool)trials[setNumber-1].visual;
  vibrotactile = (bool)trials[setNumber-1].vibrotactile;
  skin_stretch = (bool)trials[setNumber-1].skin_stretch;
  imp1.k = trials[setNumber-1].k1[trialNumber-1];
  imp1.b = trials[setNumber-1].b1[trialNumber-1];
  imp1.m = trials[setNumber-1].m1[trialNumber-1];
  imp2.k = trials[setNumber-1].k2[trialNumber-1];
  imp2.b = trials[setNumber-1].b2[trialNumber-1];
  imp2.m = trials[setNumber-1].m2[trialNumber-1];
  direction = trials[setNumber-1].direction[trialNumber-1];
  feedback_change = true;

}


void NextTrial(void) {
  // increment set number, trial number, etc.
  // and set next state as appropriate
  if (trialNumber >= nTrials) {
    if (setNumber >= nSets) {
      state = experimentDone;
    } else {
      if (set_type == experiment) {
        // finished this feedback type, so take a break;
        state = betweenSets;
      } else if (set_type == training) {
        // take breaks between all training sets
        state = betweenTraining;
      } else {
        // finished practice set, move on to experiment set (no break)
        state = instructions;
      }
      trialNumber = 1;
      setNumber++;
      task = (taskType)trials[setNumber-1].task;
      nTrials = trials[setNumber-1].nTrials;
      variable = trials[setNumber-1].variable;
      set_type = (setType)trials[setNumber-1].set_type;
      if (set_type == training) {
        trialLength = trialLengthT;
      } else {
        trialLength = trialLengthPE;
      }
      visual = (bool)trials[setNumber-1].visual;
      vibrotactile = (bool)trials[setNumber-1].vibrotactile;
      skin_stretch = (bool)trials[setNumber-1].skin_stretch;
      imp1.k = trials[setNumber-1].k1[trialNumber-1];
      imp1.b = trials[setNumber-1].b1[trialNumber-1];
      imp1.m = trials[setNumber-1].m1[trialNumber-1];
      imp2.k = trials[setNumber-1].k2[trialNumber-1];
      imp2.b = trials[setNumber-1].b2[trialNumber-1];
      imp2.m = trials[setNumber-1].m2[trialNumber-1];
      direction = trials[setNumber-1].direction[trialNumber-1];
      feedback_change = true;
    }
  } else {
    trialNumber++;
    imp1.k = trials[setNumber-1].k1[trialNumber-1];
    imp1.b = trials[setNumber-1].b1[trialNumber-1];
    imp1.m = trials[setNumber-1].m1[trialNumber-1];
    imp2.k = trials[setNumber-1].k2[trialNumber-1];
    imp2.b = trials[setNumber-1].b2[trialNumber-1];
    imp2.m = trials[setNumber-1].m2[trialNumber-1];
    direction = trials[setNumber-1].direction[trialNumber-1];
    state = waitingForUser;
  }
  change_state = true;
}


void PreviousTrial(void) {
  // decrement set number, trial number, etc.
  // and set state as appropriate
  if (state == experimentDone) {
    // trial number was not incremented before, so don't decrement now.
    // just change state.
    state = waitingForUser;
  } else if (trialNumber > 1) {
    // staying within the same set; just update trial information
    trialNumber--;
    imp1.k = trials[setNumber-1].k1[trialNumber-1];
    imp1.b = trials[setNumber-1].b1[trialNumber-1];
    imp1.m = trials[setNumber-1].m1[trialNumber-1];
    imp2.k = trials[setNumber-1].k2[trialNumber-1];
    imp2.b = trials[setNumber-1].b2[trialNumber-1];
    imp2.m = trials[setNumber-1].m2[trialNumber-1];
    direction = trials[setNumber-1].direction[trialNumber-1];
  } else if (setNumber > 1) {
    // changing sets
    // get set info first, so we know how many trials are in the set
    setNumber--;
    task = (taskType)trials[setNumber-1].task;
    nTrials = trials[setNumber-1].nTrials;
    variable = trials[setNumber-1].variable;
    set_type = (setType)trials[setNumber-1].set_type;
    if (set_type == training) {
      trialLength = trialLengthT;
    } else {
      trialLength = trialLengthPE;
    }
    visual = (bool)trials[setNumber-1].visual;
    vibrotactile = (bool)trials[setNumber-1].vibrotactile;
    skin_stretch = (bool)trials[setNumber-1].skin_stretch;
    // go to last trial of set
    trialNumber = nTrials;
    // get trial info
    imp1.k = trials[setNumber-1].k1[trialNumber-1];
    imp1.b = trials[setNumber-1].b1[trialNumber-1];
    imp1.m = trials[setNumber-1].m1[trialNumber-1];
    imp2.k = trials[setNumber-1].k2[trialNumber-1];
    imp2.b = trials[setNumber-1].b2[trialNumber-1];
    imp2.m = trials[setNumber-1].m2[trialNumber-1];
    direction = trials[setNumber-1].direction[trialNumber-1];
    feedback_change = true;
  } else {
    // already at first set, first trial; do nothing
  }
  // if first trial of set, display instructions again
  if (trialNumber == 1) {
    state = instructions;
  } else {
    state = waitingForUser;
  }
  change_state = true;
}


void ButtonPress(void) {
  // advance state, as appropriate
  switch (state) {

    case instructions:
      // signal trial start
      start = true;
      break;

    case waitingForUser:
      // signal trial start
      start = true;
      break;

    case rating:
      // nothing happens here; state is advanced when number key is pressed
      break;

    case runningTrial:
      // during feedback training, button press ends trial.
      // otherwise, nothing happens here; trial ends when time is up.
      if (set_type == training) {
        reset = true;
        state = waitingForUser;
        NextTrial();
      }
      break;

    case betweenTraining:
      // require debug mode to continue, so that the user can't do it themselves
      if (debug_mode) {
        state = instructions;
        debug_mode = false;
      }
      break;

    case betweenSets:
      // require debug mode to continue, so that the user can't do it themselves
      if (debug_mode) {
        state = instructions;
        debug_mode = false;
      }
      break;

    case experimentDone:
      // nothing happens here; sit on done until exit
      break;

    default:
      // this should never happen...
      ROS_WARN("Unknown state.");
  }
}


bool handleButtonPress(std_srvs::Empty::Request  &req,
                       std_srvs::Empty::Response &res )
{
  ButtonPress();
  return true;
}


void keyPressCallback(const std_msgs::Int32::ConstPtr& msg)
{
  key = msg->data;
  ROS_INFO("Received key press %d.",key);
  switch (key) {

    case KEY_ESC:
      ROS_INFO("Quit message received.");
      ros::shutdown();
      break;

    case KEY_1:
    case KEY_2:
    case KEY_3:
    case KEY_4:
    case KEY_5:
      if (state == rating) {

        // save rating - rating is equal to key number minus 48
        FILE *output_file;
        char fname[60]; // file name plus path

        // Open file for writing
        sprintf(fname,"data/Subject%02d/all_ratings.m",subjectNumber);
        output_file = fopen(fname,"a");

        fprintf(output_file, "ratings(%d,%d) = %d;\n",setNumber,trialNumber,(key-48));
        fprintf(output_file, "k(%d,%d) = %f;\n",setNumber,trialNumber,imp1.k);
        fprintf(output_file, "b(%d,%d) = %f;\n",setNumber,trialNumber,imp1.b);
        fprintf(output_file, "visual(%d,%d) = %d;\n",setNumber,trialNumber,visual);
        fprintf(output_file, "vibrotactile(%d,%d) = %d;\n",setNumber,trialNumber,vibrotactile);
        fprintf(output_file, "skin_stretch(%d,%d) = %d;\n",setNumber,trialNumber,skin_stretch);
        fprintf(output_file, "\n");
        
        fclose(output_file);

        NextTrial();
      }
      break;

    case KEY_7:
    case KEY_8:
    case KEY_9:
    case KEY_0:
    case KEY_I:
    case KEY_O:
      if (tunable_PD) {
        change_PD_gains = true;
        gain_key = key;
      }
      break;

    case KEY_R:
      if (debug_mode) {
        reset = true;
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

    case KEY_V:
      // toggle vibrotactile
      vibrotactile = !vibrotactile;
      feedback_change = true;
      break;

    case KEY_G:
      // toggle visual feedback (G for graphics!)
      visual = !visual;
      break;

    case KEY_S:
      // toggle skin stretch feedback
      skin_stretch = !skin_stretch;
      feedback_change = true;
      break;

    case KEY_TILDE:
      debug_mode = !debug_mode;
      break;

    case KEY_N:
      if (debug_mode) {
        if (state == runningTrial) {
          // end trial
          reset = true;
        }
        // skip next to trial
        NextTrial();
        debug_mode = false;
      }
      break;

    case KEY_PLUS:
      if (debug_mode) {
        // skip to next set
        if (state == runningTrial) {
          // end trial
          reset = true;
        }
        trialNumber = nTrials;
        NextTrial();
        debug_mode = false;
      }
      break;

    case KEY_MINUS:
      if (debug_mode) {
        // skip to beginning of set or back to previous set
        if (state == runningTrial) {
          reset = true;
          state = waitingForUser;
        }
        if (trialNumber == 1) {
          if (setNumber > 1) {
            // go to previous set, last trial (to change feedback conditions, etc)
            PreviousTrial();
            // go to first trial
            trialNumber = 2;
            PreviousTrial();
          }
        } else {
          trialNumber = 2;
          PreviousTrial();
        }
        debug_mode = false;
        state = instructions;
      }
      break;

    case KEY_P:
      if (debug_mode) {
        if (state == runningTrial) {
          // reset trial (to waitingForUser state before trial)
          reset = true;
          state = waitingForUser;
        } else {
          // skip back to previous trial
          PreviousTrial();
        }
        debug_mode = false;
      }
      break;

    default:
      ROS_INFO("Pressing %d doesn't do anything.", key);
      break;
  }

}


int main(int argc, char **argv) {

  // initialize gains
  imp1.m = 1.5; // mass of virtual prosthesis
  imp2.m = 1.5;// mass of moving object in LI task
  imp1.k = 20; // stiffness of virtual prosthesis
  imp2.k = 20; // stiffness of moving object in LI task
  imp1.b = 2.5; // viscosity of virtual prosthesis
  imp2.b = 2.5; // viscosity of moving object in LI task


  std::cout << "Subject number? ";
  std::cin >> subjectNumber;

  // get set/trial info
  GetTrialInfo(subjectNumber);
  ROS_INFO("Trial info loaded.");

  // set up ROS stuff
  ros::init(argc, argv, "experiment_manager");
  ros::NodeHandle n;
  ros::Publisher graphics_info_pub = n.advertise<simulation_impedance_experiment::GraphicsInfo>("graphics_info", 1000);
  ros::Publisher gains_pub = n.advertise<simulation_impedance_experiment::Gains>("gains", 1000);
  ros::ServiceClient client_set_graphics_mode = n.serviceClient<simulation_impedance_experiment::SetGraphicsMode>("set_graphics_mode");
  ros::ServiceClient client_change_state = n.serviceClient<simulation_impedance_experiment::ChangeState>("change_state");
  ros::ServiceClient client_start_simulation = n.serviceClient<simulation_impedance_experiment::StartSimulation>("start_simulation");
  ros::ServiceClient client_reset_simulation = n.serviceClient<std_srvs::Empty>("reset_simulation");
  ros::ServiceClient client_set_haptic_feedback = n.serviceClient<simulation_impedance_experiment::SetHapticFeedback>("set_haptic_feedback");
  ros::ServiceClient client_set_PD_gains = n.serviceClient<simulation_impedance_experiment::IntegerRequest>("set_PD_gains");
  ros::ServiceServer server_button_press = n.advertiseService("handle_button_press", handleButtonPress);

  // subscriber for receiving key presses
  ros::Subscriber sub_key_press = n.subscribe<std_msgs::Int32>("key_press", 1000, keyPressCallback);

  ros::Rate loop_rate(100); // publishing rate in Hz
  ros::Time t;
  ros::Time told;
  ros::Duration deltat;

  // declare message types
  simulation_impedance_experiment::GraphicsInfo msg_graphics_info;
  simulation_impedance_experiment::Gains msg_gains;

  // declare services
  simulation_impedance_experiment::SetGraphicsMode srv_set_graphics_mode;
  simulation_impedance_experiment::ChangeState srv_change_state;
  simulation_impedance_experiment::StartSimulation srv_start_simulation;
  std_srvs::Empty srv_reset_simulation;
  simulation_impedance_experiment::SetHapticFeedback srv_set_haptic_feedback;
  simulation_impedance_experiment::IntegerRequest srv_set_PD_gains;


  // set graphics info for drawing sine wave. hard-coded for now, just to test. Later I will do something useful with it.
  double phase = 0.0;
  double limitL = -1.0;
  double limitR = 1.5;


  ros::Time t_start = ros::Time::now();

  // loop: define and publish data

  while (ros::ok()) {

    // get timestamp and period
    told = t;
    t = ros::Time::now();
    deltat = t - told;
    msg_graphics_info.t = t;
    msg_graphics_info.deltat = deltat;
    msg_gains.t = t;

    // put info for drawing sine wave into graphics info message
    msg_graphics_info.task = (int)task;
    msg_graphics_info.limitL = limitL;
    msg_graphics_info.limitR = limitR;
    msg_graphics_info.phase = phase;

    // put gain info into gains messages
    msg_gains.stiffness1 = imp1.k;
    msg_gains.damping1 = imp1.b;
    msg_gains.mass1 = imp1.m;
    msg_gains.stiffness2 = imp2.k;
    msg_gains.damping2 = imp2.b;
    msg_gains.mass2 = imp2.m;

    // publish messages
    graphics_info_pub.publish(msg_graphics_info);
    gains_pub.publish(msg_gains);

    // do state-appropriate things!
    switch (state) {

      case instructions:
        ROS_DEBUG("State: instructions");
        // do nothing here; we're waiting for the user to press the button.
        break;

      case waitingForUser:
        ROS_DEBUG("State: waitingForUser");
        // do nothing here; we're waiting for the user to press the button.
        break;

      case runningTrial:
        ROS_DEBUG("State: runningTrial");
        // keep track of time, check for end of trial
        if ((t - t_start).toSec() >= trialLength) {
          reset = true;
          if (set_type == training) {
            state = waitingForUser;
            NextTrial();
          } else {
            state = rating;
          }
        }
        break;

      case rating:
        ROS_DEBUG("State: rating");
        // do nothing here; we're waiting for the user to enter a number.
        break;

      case betweenSets:
        ROS_DEBUG("State: betweenSets");
        // do nothing here; we're waiting for the experiment to advance the state
        break;

      case betweenTraining:
        ROS_DEBUG("State: betweenTraining");
        // do nothing here; we're waiting for the experiment to advance the state
        break;

      case experimentDone:
        ROS_DEBUG("State: experimentDone");
        // do nothing here; the experiment is over.
        break;

      default:
        // this should never happen...
        ROS_WARN("Unknown state.");
    }

    if (start) {
      // start simulation
      srv_start_simulation.request.task = (int)task;
      srv_start_simulation.request.subjectNumber = subjectNumber;
      srv_start_simulation.request.trialNumber = trialNumber;
      srv_start_simulation.request.setNumber = setNumber;
      srv_start_simulation.request.set_type = (int)set_type;
      srv_start_simulation.request.visual = visual;
      srv_start_simulation.request.vibrotactile = vibrotactile;
      srv_start_simulation.request.skin_stretch = skin_stretch;
      srv_start_simulation.request.direction = direction;
      
      if (client_start_simulation.call(srv_start_simulation)) {
        t_start = ros::Time::now();
        ROS_INFO("Started simulation.");
      } else {
        ROS_ERROR("Failed to call service start_simulation.");
      }
      start = false;
      state = runningTrial;
    }

    if (reset) {
      // reset simulation
      if (client_reset_simulation.call(srv_reset_simulation)) {
        ROS_INFO("Reset simulation.");
      } else {
        ROS_ERROR("Failed to call service reset_simulation.");
      }
      reset = false;
    }

    if (feedback_change) {
      // if change to haptic feedback mode, tell daq node
      srv_set_haptic_feedback.request.vibrotactile = vibrotactile;
      srv_set_haptic_feedback.request.skin_stretch = skin_stretch;
      if (client_set_haptic_feedback.call(srv_set_haptic_feedback)) {
        if (vibrotactile) {
          ROS_INFO("Set vibrotactile feedback on.");
        } else {
          ROS_INFO("Set vibrotactile feedback off.");
        }
        if (skin_stretch) {
          ROS_INFO("Set skin stretch feedback on.");
        } else {
          ROS_INFO("Set skin stretch feedback off.");
        }
      } else {
        ROS_ERROR("Failed to call service set_haptic_feedback.");
      }
      feedback_change = false;
    }

    // if state change, tell graphics node
    if ((srv_change_state.request.state != (int)state) || (srv_change_state.request.trial_number != trialNumber) || change_state) {
      srv_change_state.request.state = (int)state;
      srv_change_state.request.trial_number = trialNumber;
      srv_change_state.request.nTrials = nTrials;
      srv_change_state.request.trial_length = trialLength;
      srv_change_state.request.set_type = int(set_type);
      srv_change_state.request.visual = visual;
      srv_change_state.request.vibrotactile = vibrotactile;
      srv_change_state.request.skin_stretch = skin_stretch;
      if (client_change_state.call(srv_change_state)) {
        ROS_INFO("Instructions type set.");
      } else {
        ROS_ERROR("Failed to call service change_state.");
      }
      change_state = false;
    }

    if ((srv_set_graphics_mode.request.debug_mode != debug_mode) || (srv_set_graphics_mode.request.showforce != visual)) {
      // change graphics for debug mode or visual feedback
      srv_set_graphics_mode.request.debug_mode = debug_mode;
      srv_set_graphics_mode.request.showforce = visual; // show force vector if visual feedback is active
      if (client_set_graphics_mode.call(srv_set_graphics_mode)) {
        ROS_INFO("Graphics mode set.");
      } else {
        ROS_ERROR("Failed to call service set_graphics_mode.");
      }
    }

    // set skin stretch motor PD gains, if applicable
    if (change_PD_gains) {
      srv_set_PD_gains.request.value = gain_key; // just directly passing the key press
      if (client_set_PD_gains.call(srv_set_PD_gains)) {
        ROS_INFO("Skin stretch PD gains set.");
      } else {
        ROS_ERROR("Failed to call service set_PD_gains.");
      }
      change_PD_gains = false;
    }



    ros::spinOnce();
    loop_rate.sleep();

  }

  return 0;
}
