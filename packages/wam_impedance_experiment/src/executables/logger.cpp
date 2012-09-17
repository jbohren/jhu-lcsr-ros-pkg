#include "ros/ros.h"
#include "ros/package.h"
//#include "wam_impedance_experiment/ExperimentState.h"
#include "wam_impedance_experiment/TaskType.h"
#include "wam_impedance_experiment/SetType.h"
#include "wam_impedance_experiment/TrajectoryType.h"
//#include "wam_impedance_experiment/Gains.h"
#include "wam_impedance_experiment/DAQinput.h"
#include "wam_impedance_experiment/StartLogging.h"
#include "wam_impedance_experiment/Int.h"
//#include "wam_impedance_experiment/ProjectileInfo.h"
#include <wam_impedance_experiment/TrajectoryInfo.h>
#include <wam_impedance_experiment/ControllerData.h>
#include <sensor_msgs/JointState.h>
#include "std_srvs/Empty.h"
#include "std_msgs/Empty.h"

bool recordData = false;
std::string pathname; // path where data files are stored

// verbose logging saves all of the information to file.
// when this option is set to false, the logger still keeps track of all the information, but
// only some of it is written to file.
bool verbose = false;

// for each topic that contains data to be logged, define a structure to store data and a counter

const int MAX_SAMPLES = 200000;

typedef struct {
  double t[MAX_SAMPLES];
  double k1[MAX_SAMPLES];
  double k2[MAX_SAMPLES];
  double b1[MAX_SAMPLES];
  double b2[MAX_SAMPLES];
  double m1[MAX_SAMPLES];
  double m2[MAX_SAMPLES];
} gainsStruct;

gainsStruct data_gains;
int count_gains = 0;

typedef struct {
  double t[MAX_SAMPLES];
  double x_p[5][MAX_SAMPLES];
  double impulse[MAX_SAMPLES];
} projectilesStruct;

projectilesStruct data_projectiles;
int count_projectiles = 0;

typedef struct {
  double t[MAX_SAMPLES];
  double Fu_counts[MAX_SAMPLES];
  double Fu_raw[MAX_SAMPLES];
  double Fu[MAX_SAMPLES];
  double Fenv_counts[MAX_SAMPLES];
  double Fenv_raw[MAX_SAMPLES];
  double Fenv[MAX_SAMPLES];
  double pot1_counts[MAX_SAMPLES];
  double pot1_raw[MAX_SAMPLES];
  double pot1[MAX_SAMPLES];
  double pot2_counts[MAX_SAMPLES];
  double pot2_raw[MAX_SAMPLES];
  double pot2[MAX_SAMPLES];
  double zeroforce1[MAX_SAMPLES];
  double zeroforce2[MAX_SAMPLES];
  double forcescale1[MAX_SAMPLES];
  double forcescale2[MAX_SAMPLES];
} inputStruct;

inputStruct data_input;
int count_input = 0;

typedef struct {
  double t[MAX_SAMPLES];
  std::string name[MAX_SAMPLES];
  double data[7][MAX_SAMPLES];
} robotStateStruct;

robotStateStruct data_joint_positions;
int count_joint_positions = 0;

typedef struct {
  double t[MAX_SAMPLES];
  double deltat[MAX_SAMPLES];
  double position[7][MAX_SAMPLES];
  double velocity[7][MAX_SAMPLES];
  double torque_gc[7][MAX_SAMPLES];
  double torque_ct[7][MAX_SAMPLES];
  double torque_imp[7][MAX_SAMPLES];
  double torque_imp_null[7][MAX_SAMPLES];
  double torque_imp_null_fric[7][MAX_SAMPLES];
  double torque_external[7][MAX_SAMPLES];
  double torque[7][MAX_SAMPLES];
  double setpointRn[7][MAX_SAMPLES];
  double setpointRnvelocity[7][MAX_SAMPLES];
  double setpoint_null[7][MAX_SAMPLES];
  double setpointSE3velocity[6][MAX_SAMPLES];
  double Kp_Rn[7][MAX_SAMPLES];
  double Kd_Rn[7][MAX_SAMPLES];
  double Kp_SE3[6][MAX_SAMPLES];
  double Kd_SE3[6][MAX_SAMPLES];
  double erroraxis[3][MAX_SAMPLES];
  double Kp_null[MAX_SAMPLES];
  double Kd_null[MAX_SAMPLES];
  double friction_gain[MAX_SAMPLES];
  double impulse[MAX_SAMPLES];
  double forceExternal[MAX_SAMPLES];
  double pose[4][4][MAX_SAMPLES];
  double M[3][3][MAX_SAMPLES];
  double M_OS[6][6][MAX_SAMPLES];
  double M_JS[7][7][MAX_SAMPLES];
  double setpointSE3[4][4][MAX_SAMPLES];
  int controllerNumber[MAX_SAMPLES];
  double errorangle[MAX_SAMPLES];
  double mass[MAX_SAMPLES];
  double tracking_goal[MAX_SAMPLES];
} controllerDataStruct;

controllerDataStruct data_controller_0;
controllerDataStruct data_controller_1;
int count_controller_data_0 = 0;
int count_controller_data_1 = 0;

typedef struct {
  double t[MAX_SAMPLES];
  int trajectory[MAX_SAMPLES];
  double qs[7][MAX_SAMPLES];
  double vws[6][MAX_SAMPLES];
  double Rts[4][4][MAX_SAMPLES];
  double maxCommandSpeed[MAX_SAMPLES];
} trajectoryStruct;

trajectoryStruct data_trajectory_0;
trajectoryStruct data_trajectory_1;
int count_trajectory_0 = 0;
int count_trajectory_1 = 0;

typedef struct {
  double t[MAX_SAMPLES];
  double x_a[MAX_SAMPLES];
  double x_d1[MAX_SAMPLES];
  double x_d2[MAX_SAMPLES];
  double xd_a[MAX_SAMPLES];
  double xd_d1[MAX_SAMPLES];
  double xd_d2[MAX_SAMPLES];
  double x_t[MAX_SAMPLES];
  double Fenv[MAX_SAMPLES];
} dataStruct;

dataStruct data;
int count_data = 0;

typedef struct {
  int subjectNumber;
  int trialNumber;
  int setNumber;
  int set_type;       // practice (0), experiment (1), or training (2)
  int task;           // forcemin (1) or tracking (0)
  double admittance;
  double deadband;
  double direction;   // direction of sine wave
} trialInfoStruct;

trialInfoStruct data_trial_info;


void toggleVerboseLogging(const std_msgs::Empty::ConstPtr& msg) {
  verbose = !verbose;
  if (verbose) {
    ROS_INFO("Logger: Verbose logging activated.");
  } else {
    ROS_INFO("Logger: Verbose logging deactivated.");
  }
}


/*
void GainsCallback(const wam_impedance_experiment::Gains::ConstPtr& msg) {
  if (recordData) {
    data_gains.t[count_gains] = msg->t.toSec();
    data_gains.k1[count_gains] = msg->stiffness1;
    data_gains.b1[count_gains] = msg->damping1;
    data_gains.m1[count_gains] = msg->mass1;
    data_gains.k2[count_gains] = msg->stiffness2;
    data_gains.b2[count_gains] = msg->damping2;
    data_gains.m2[count_gains] = msg->mass2;
    count_gains++;
  }

  if (count_gains > MAX_SAMPLES) {
    ROS_ERROR("Logger: Data logging of gains exceeded array size. Overwriting data array.");
    count_gains = 0;
  }
}
*/


/*
void ProjectilesCallback(const wam_impedance_experiment::ProjectileInfo::ConstPtr& msg) {
  if (recordData) {
    data_projectiles.t[count_projectiles] = msg->t.toSec();
    for (int i=0;i<5;i++) {
      data_projectiles.x_p[i][count_projectiles] = msg->x_p[i];
    }
    data_projectiles.impulse[count_projectiles] = msg->impulse;
    count_projectiles++;
  }

  if (count_projectiles > MAX_SAMPLES) {
    ROS_ERROR("Logger: Data logging of projectile data exceeded array size. Overwriting data array.");
    count_projectiles = 0;
  }
}
*/


void UserInputCallback(const wam_impedance_experiment::DAQinput::ConstPtr& msg) {
  if (recordData) {
    data_input.t[count_input] = msg->t.toSec();
    data_input.Fu_counts[count_input] = msg->force1_counts;
    data_input.Fu_raw[count_input] = msg->force1_raw;
    data_input.Fu[count_input] = msg->force1;
    data_input.Fenv_counts[count_input] = msg->force2_counts;
    data_input.Fenv_raw[count_input] = msg->force2_raw;
    data_input.Fenv[count_input] = msg->force2;
    data_input.pot1_counts[count_input] = msg->pot1_counts;
    data_input.pot1_raw[count_input] = msg->pot1_raw;
    data_input.pot1[count_input] = msg->pot1;
    data_input.pot2_counts[count_input] = msg->pot2_counts;
    data_input.pot2_raw[count_input] = msg->pot2_raw;
    data_input.pot2[count_input] = msg->pot2;
    data_input.zeroforce1[count_input] = msg->zeroforce1;
    data_input.zeroforce2[count_input] = msg->zeroforce2;
    data_input.forcescale1[count_input] = msg->forcescale1;
    data_input.forcescale2[count_input] = msg->forcescale2;
    count_input++;
  }

  if (count_input > MAX_SAMPLES) {
    ROS_ERROR("Logger: Data logging of DAQ input exceeded array size. Overwriting data array.");
    count_input = 0;
  }
}

void JointStateCallback(const sensor_msgs::JointState::ConstPtr& msg) {
  if (recordData) {
    data_joint_positions.t[count_joint_positions] = msg->header.stamp.toSec();
    data_joint_positions.name[count_joint_positions] = msg->name[0];
    for (int i=0;i<7;i++) {
      data_joint_positions.data[i][count_joint_positions] = msg->position[i];
    }
    count_joint_positions++;
  }

  if (count_joint_positions > MAX_SAMPLES) {
    ROS_ERROR("Logger: Data logging of joint positions exceeded array size. Overwriting data array.");
    count_joint_positions = 0;
  }
}

void ControllerDataCallback(const wam_impedance_experiment::ControllerData::ConstPtr& msg) {
  if (recordData) {
    if ((msg->name == "robot0") || (msg->name == "left")) {

      data_controller_0.t[count_controller_data_0] = msg->t.toSec();
      data_controller_0.deltat[count_controller_data_0] = msg->deltat.toSec();
      for (int i=0;i<7;i++) {
        data_controller_0.position[i][count_controller_data_0] = msg->position[i];
        data_controller_0.velocity[i][count_controller_data_0] = msg->velocity[i];
        data_controller_0.torque_gc[i][count_controller_data_0] = msg->torque_gc[i];
        data_controller_0.torque_ct[i][count_controller_data_0] = msg->torque_ct[i];
        data_controller_0.torque_imp[i][count_controller_data_0] = msg->torque_imp[i];
        data_controller_0.torque_imp_null[i][count_controller_data_0] = msg->torque_imp_null[i];
        data_controller_0.torque_imp_null_fric[i][count_controller_data_0] = msg->torque_imp_null_fric[i];
        data_controller_0.torque_external[i][count_controller_data_0] = msg->torque_external[i];
        data_controller_0.torque[i][count_controller_data_0] = msg->torque[i];
        data_controller_0.setpointRn[i][count_controller_data_0] = msg->setpointRn[i];
        data_controller_0.setpointRnvelocity[i][count_controller_data_0] = msg->setpointRnvelocity[i];
        data_controller_0.setpoint_null[i][count_controller_data_0] = msg->setpoint_null[i];
        data_controller_0.errorangle[count_controller_data_0] = msg->errorangle;
        data_controller_0.mass[count_controller_data_0] = msg->m_y;
        if (i<3) {
          data_controller_0.erroraxis[i][count_controller_data_0] = msg->erroraxis[i];
        }
        if (i<6) {
          data_controller_0.setpointSE3velocity[i][count_controller_data_0] = msg->setpointSE3velocity[i];
          data_controller_0.Kp_SE3[i][count_controller_data_0] = msg->Kp_SE3[i];
          data_controller_0.Kd_SE3[i][count_controller_data_0] = msg->Kd_SE3[i];
        }
        data_controller_0.Kp_Rn[i][count_controller_data_0] = msg->Kp_Rn[i];
        data_controller_0.Kd_Rn[i][count_controller_data_0] = msg->Kd_Rn[i];
      }
      data_controller_0.Kp_null[count_controller_data_0] = msg->Kp_null;
      data_controller_0.Kd_null[count_controller_data_0] = msg->Kd_null;
      data_controller_0.friction_gain[count_controller_data_0] = msg->friction_gain;
      data_controller_0.impulse[count_controller_data_0] = msg->impulse;
      data_controller_0.forceExternal[count_controller_data_0] = msg->forceExternal;
      for (int i=0;i<7;i++) {
        for (int j=0;j<7;j++) {
          if ((i<4) && (j<4)) {
            data_controller_0.pose[i][j][count_controller_data_0] = msg->pose.data[(msg->pose.layout.data_offset) + (msg->pose.layout.dim[0].stride)*i + j];
            data_controller_0.setpointSE3[i][j][count_controller_data_0] = msg->setpointSE3.data[(msg->setpointSE3.layout.data_offset) + (msg->setpointSE3.layout.dim[0].stride)*i + j];
          }
          if ((i<3) && (j<3)) {
            data_controller_0.M[i][j][count_controller_data_0] = msg->M.data[(msg->M.layout.data_offset) + (msg->M.layout.dim[0].stride)*i + j];
          }
          if ((i<6) && (j<6)) {
            data_controller_0.M_OS[i][j][count_controller_data_0] = msg->M_OS.data[(msg->M_OS.layout.data_offset) + (msg->M_OS.layout.dim[0].stride)*i + j];
          }
          data_controller_0.M_JS[i][j][count_controller_data_0] = msg->M_JS.data[(msg->M_JS.layout.data_offset) + (msg->M_JS.layout.dim[0].stride)*i + j];
        }
      }

      data_controller_0.controllerNumber[count_controller_data_0] = msg->controller;
      data_controller_0.tracking_goal[count_controller_data_0] = msg->tracking_goal;
      count_controller_data_0++;

      if (count_controller_data_0 > MAX_SAMPLES) {
        ROS_ERROR("Logger: Data logging of joint torques exceeded array size (robot 0). Overwriting data array.");
        count_controller_data_0 = 0;
      }

    } else if ((msg->name == "robot1") || (msg->name == "right")) {

      data_controller_1.t[count_controller_data_1] = msg->t.toSec();
      data_controller_1.deltat[count_controller_data_1] = msg->deltat.toSec();
      for (int i=0;i<7;i++) {
        data_controller_1.position[i][count_controller_data_1] = msg->position[i];
        data_controller_1.velocity[i][count_controller_data_1] = msg->velocity[i];
        data_controller_1.torque_gc[i][count_controller_data_1] = msg->torque_gc[i];
        data_controller_1.torque_ct[i][count_controller_data_1] = msg->torque_ct[i];
        data_controller_1.torque_imp[i][count_controller_data_1] = msg->torque_imp[i];
        data_controller_1.torque_imp_null[i][count_controller_data_1] = msg->torque_imp_null[i];
        data_controller_1.torque_imp_null_fric[i][count_controller_data_1] = msg->torque_imp_null_fric[i];
        data_controller_1.torque_external[i][count_controller_data_1] = msg->torque_external[i];
        data_controller_1.torque[i][count_controller_data_1] = msg->torque[i];
        data_controller_1.setpointRn[i][count_controller_data_1] = msg->setpointRn[i];
        data_controller_1.setpointRnvelocity[i][count_controller_data_1] = msg->setpointRnvelocity[i];
        data_controller_1.setpoint_null[i][count_controller_data_1] = msg->setpoint_null[i];
        data_controller_1.errorangle[count_controller_data_1] = msg->errorangle;
        data_controller_1.mass[count_controller_data_1] = msg->m_y;
        if (i<3) {
          data_controller_1.erroraxis[i][count_controller_data_1] = msg->erroraxis[i];
        }
        if (i<6) {
          data_controller_1.setpointSE3velocity[i][count_controller_data_1] = msg->setpointSE3velocity[i];
          data_controller_1.Kp_SE3[i][count_controller_data_1] = msg->Kp_SE3[i];
          data_controller_1.Kd_SE3[i][count_controller_data_1] = msg->Kd_SE3[i];
        }
        data_controller_1.Kp_Rn[i][count_controller_data_1] = msg->Kp_Rn[i];
        data_controller_1.Kd_Rn[i][count_controller_data_1] = msg->Kd_Rn[i];
      }
      data_controller_1.Kp_null[count_controller_data_1] = msg->Kp_null;
      data_controller_1.Kd_null[count_controller_data_1] = msg->Kd_null;
      data_controller_1.friction_gain[count_controller_data_1] = msg->friction_gain;
      data_controller_1.impulse[count_controller_data_1] = msg->impulse;
      data_controller_1.forceExternal[count_controller_data_1] = msg->forceExternal;
      for (int i=0;i<7;i++) {
        for (int j=0;j<7;j++) {
          if ((i<4) && (j<4)) {
            data_controller_1.pose[i][j][count_controller_data_0] = msg->pose.data[(msg->pose.layout.data_offset) + (msg->pose.layout.dim[0].stride)*i + j];
            data_controller_1.setpointSE3[i][j][count_controller_data_0] = msg->setpointSE3.data[(msg->setpointSE3.layout.data_offset) + (msg->setpointSE3.layout.dim[0].stride)*i + j];
          }
          if ((i<3) && (j<3)) {
            data_controller_1.M[i][j][count_controller_data_0] = msg->M.data[(msg->M.layout.data_offset) + (msg->M.layout.dim[0].stride)*i + j];
          }
          if ((i<6) && (j<6)) {
            data_controller_1.M_OS[i][j][count_controller_data_0] = msg->M_OS.data[(msg->M_OS.layout.data_offset) + (msg->M_OS.layout.dim[0].stride)*i + j];
          }
          data_controller_1.M_JS[i][j][count_controller_data_0] = msg->M_JS.data[(msg->M_JS.layout.data_offset) + (msg->M_JS.layout.dim[0].stride)*i + j];
        }
      }

      data_controller_1.controllerNumber[count_controller_data_1] = msg->controller;
      count_controller_data_1++;

      if (count_controller_data_1 > MAX_SAMPLES) {
        ROS_ERROR("Logger: Data logging of joint torques exceeded array size (robot 1). Overwriting data array.");
        count_controller_data_1 = 0;
      }

    } else {
      ROS_ERROR_STREAM("Logger: Unknown robot name \"" << msg->name << "\".");
    }
  }
}

void TrajectoryCallback(const wam_impedance_experiment::TrajectoryInfo::ConstPtr& msg) {
  if (recordData) {
    if ((msg->name == "robot0") || (msg->name == "left")) {

      data_trajectory_0.t[count_trajectory_0] = msg->t.toSec();
      data_trajectory_0.trajectory[count_trajectory_0] = (int)msg->trajectoryType;
      data_trajectory_0.maxCommandSpeed[count_trajectory_0] = msg->maxCommandSpeed;
      for (int i=0;i<7;i++) {
        data_trajectory_0.qs[i][count_trajectory_0] = msg->setpointRn[i];
      }
      for (int i=0;i<6;i++) {
        data_trajectory_0.vws[i][count_trajectory_0] = msg->velocitySE3[i];
      }
      for (int i=0;i<4;i++) {
        for (int j=0;j<4;j++) {
          data_trajectory_0.Rts[i][j][count_trajectory_0] = msg->setpointSE3.data[(msg->setpointSE3.layout.data_offset) + (msg->setpointSE3.layout.dim[0].stride)*i + j];
        }
      }
      count_trajectory_0++;

      if (count_trajectory_0 > MAX_SAMPLES) {
        ROS_ERROR("Logger: Data logging of commanded trajectory exceeded array size. Overwriting data array.");
        count_trajectory_0 = 0;
      }

    } else if ((msg->name == "robot1") || (msg->name == "right")) {

      data_trajectory_1.t[count_trajectory_1] = msg->t.toSec();
      data_trajectory_1.trajectory[count_trajectory_1] = (int)msg->trajectoryType;
      for (int i=0;i<7;i++) {
        data_trajectory_1.qs[i][count_trajectory_1] = msg->setpointRn[i];
      }
      for (int i=0;i<6;i++) {
        data_trajectory_1.vws[i][count_trajectory_1] = msg->velocitySE3[i];
      }
      for (int i=0;i<4;i++) {
        for (int j=0;j<4;j++) {
          data_trajectory_1.Rts[i][j][count_trajectory_1] = msg->setpointSE3.data[(msg->setpointSE3.layout.data_offset) + (msg->setpointSE3.layout.dim[0].stride)*i + j];
        }
      }
      count_trajectory_1++;

      if (count_trajectory_1 > MAX_SAMPLES) {
        ROS_ERROR("Logger: Data logging of commanded trajectory exceeded array size. Overwriting data array.");
        count_trajectory_1 = 0;
      }

    } else {
      ROS_ERROR_STREAM("Logger: Unknown robot name \"" << msg->name << "\".");
    }
  }
}


/*
void ExperimentStateCallback(const wam_impedance_experiment::ExperimentState::ConstPtr& msg) {
  if (recordData) {
    data.t[count_data] = msg->t.toSec();
    data.x_a[count_data] = msg->x_a;
    data.x_d1[count_data] = msg->x_d1;
    data.x_d2[count_data] = msg->x_d2;
    data.xd_a[count_data] = msg->xd_a;
    data.xd_d1[count_data] = msg->xd_d1;
    data.xd_d2[count_data] = msg->xd_d2;
    data.x_t[count_data] = msg->x_t;
    data.Fenv[count_data] = msg->Fenv;
    count_data++;
  }
}
*/


void Quit(const std_msgs::Empty::ConstPtr& msg) {
  ROS_INFO("Logger: Quit message received.");
  ros::shutdown();
}


// function for writing data to file.

void WriteDataFile(int subjectNumber, int setNumber, int trialNumber, int userRating) {

	FILE *output_file;
  char fname[300]; // file name plus path

	// Open file for writing
	sprintf(fname,"%s/data/Subject%02d/Set%02d_Trial%02d_info.m",pathname.c_str(),subjectNumber,setNumber,trialNumber);
	output_file = fopen(fname,"a");
  if(output_file==NULL) {
    ROS_ERROR("Logger: Can't open file %s.",fname);
  }

  // print trial info
	fprintf(output_file, "subjectNumber = %d;\n",data_trial_info.subjectNumber);
	fprintf(output_file, "setNumber = %d;\n",data_trial_info.setNumber);
	fprintf(output_file, "trialNumber = %d;\n",data_trial_info.trialNumber);
	fprintf(output_file, "set_type = %d; %% 0 for practice, 1 for experiment, 2 for training\n",data_trial_info.set_type);
	fprintf(output_file, "task = %d; %% 0 for tracking, 1 for forcemin\n",data_trial_info.task);
	fprintf(output_file, "admittance = %f;\n",data_trial_info.admittance);
	fprintf(output_file, "deadband = %f;\n",data_trial_info.deadband);
  fprintf(output_file, "direction = %f; %% direction of sine wave\n",data_trial_info.direction);

  // print user rating
	fprintf(output_file, "rating = %d;\n",userRating);


  //
  // print impedance data...
  //
  // print time stamps
  //
  // this experiment does not use variable gains
  /*
	fprintf(output_file, "data_gains.t = [");
	for (int k=0; k<count_gains; k++) {
		fprintf(output_file,"%f, ",data_gains.t[k]);
	}
	fprintf(output_file, "];\n\n");

  // print impedance levels
  fprintf(output_file, "data_gains.k1 = [");
	for (int j=0; j<count_gains; j++) {
		fprintf(output_file,"%f, ",data_gains.k1[j]);
	}
	fprintf(output_file, "];\n\n");
	fprintf(output_file, "data_gains.b1 = [");
	for (int j=0; j<count_gains; j++) {
		fprintf(output_file,"%f, ",data_gains.b1[j]);
	}
	fprintf(output_file, "];\n\n");
	fprintf(output_file, "data_gains.m1 = [");
	for (int j=0; j<count_gains; j++) {
		fprintf(output_file,"%f, ",data_gains.m1[j]);
	}
	fprintf(output_file, "];\n\n");
  fprintf(output_file, "data_gains.k2 = [");
	for (int j=0; j<count_gains; j++) {
		fprintf(output_file,"%f, ",data_gains.k2[j]);
	}
	fprintf(output_file, "];\n\n");
	fprintf(output_file, "data_gains.b2 = [");
	for (int j=0; j<count_gains; j++) {
		fprintf(output_file,"%f, ",data_gains.b2[j]);
	}
	fprintf(output_file, "];\n\n");
	fprintf(output_file, "data_gains.m2 = [");
	for (int j=0; j<count_gains; j++) {
		fprintf(output_file,"%f, ",data_gains.m2[j]);
	}
	fprintf(output_file, "];\n\n");
  */

  //
  // print input data
  //

	fprintf(output_file, "data_input.t = [");
  for (int j=0; j<count_input; j++) {
    fprintf(output_file,"%f, ",data_input.t[j]);
  }
  fprintf(output_file, "];\n\n");

	fprintf(output_file, "data_input.Fu = [");
  for (int j=0; j<count_input; j++) {
    fprintf(output_file,"%f, ",data_input.Fu[j]);
  }
  fprintf(output_file, "];\n\n");

  if (verbose) {
    fprintf(output_file, "data_input.Fu_counts = [");
    for (int j=0; j<count_input; j++) {
      fprintf(output_file,"%f, ",data_input.Fu_counts[j]);
    }
    fprintf(output_file, "];\n\n");

    fprintf(output_file, "data_input.Fu_raw = [");
    for (int j=0; j<count_input; j++) {
      fprintf(output_file,"%f, ",data_input.Fu_raw[j]);
    }
    fprintf(output_file, "];\n\n");
  }

  if (verbose || (data_trial_info.task == 1)) {
    fprintf(output_file, "data_input.Fenv = [");
    for (int j=0; j<count_input; j++) {
      fprintf(output_file,"%f, ",data_input.Fenv[j]);
    }
    fprintf(output_file, "];\n\n");
  }

  if (verbose) {
    fprintf(output_file, "data_input.Fenv_counts = [");
    for (int j=0; j<count_input; j++) {
      fprintf(output_file,"%f, ",data_input.Fenv_counts[j]);
    }
    fprintf(output_file, "];\n\n");

    fprintf(output_file, "data_input.Fenv_raw = [");
    for (int j=0; j<count_input; j++) {
      fprintf(output_file,"%f, ",data_input.Fenv_raw[j]);
    }
    fprintf(output_file, "];\n\n");
  }

  // no potentiometers in this experiment
  /*
	fprintf(output_file, "data_input.pot1 = [");
  for (int j=0; j<count_input; j++) {
    fprintf(output_file,"%f, ",data_input.pot1[j]);
  }
  fprintf(output_file, "];\n\n");

  if (verbose) {
    fprintf(output_file, "data_input.pot1_counts = [");
    for (int j=0; j<count_input; j++) {
    fprintf(output_file,"%f, ",data_input.pot1_counts[j]);
    }
    fprintf(output_file, "];\n\n");

    fprintf(output_file, "data_input.pot1_raw = [");
    for (int j=0; j<count_input; j++) {
    fprintf(output_file,"%f, ",data_input.pot1_raw[j]);
    }
    fprintf(output_file, "];\n\n");
  }

	fprintf(output_file, "data_input.pot2 = [");
  for (int j=0; j<count_input; j++) {
    fprintf(output_file,"%f, ",data_input.pot2[j]);
  }
  fprintf(output_file, "];\n\n");

  if (verbose) {
    fprintf(output_file, "data_input.pot2_counts = [");
    for (int j=0; j<count_input; j++) {
      fprintf(output_file,"%f, ",data_input.pot2_counts[j]);
    }
    fprintf(output_file, "];\n\n");

    fprintf(output_file, "data_input.pot2_raw = [");
    for (int j=0; j<count_input; j++) {
      fprintf(output_file,"%f, ",data_input.pot2_raw[j]);
    }
    fprintf(output_file, "];\n\n");
  }
  */

  if (verbose) {
    fprintf(output_file, "data_input.zeroforce1 = [");
    for (int j=0; j<count_input; j++) {
      fprintf(output_file,"%f, ",data_input.zeroforce1[j]);
    }
    fprintf(output_file, "];\n\n");

    fprintf(output_file, "data_input.zeroforce2 = [");
    for (int j=0; j<count_input; j++) {
      fprintf(output_file,"%f, ",data_input.zeroforce2[j]);
    }
    fprintf(output_file, "];\n\n");

    fprintf(output_file, "data_input.forcescale1 = [");
    for (int j=0; j<count_input; j++) {
      fprintf(output_file,"%f, ",data_input.forcescale1[j]);
    }
    fprintf(output_file, "];\n\n");

    fprintf(output_file, "data_input.forcescale2 = [");
    for (int j=0; j<count_input; j++) {
      fprintf(output_file,"%f, ",data_input.forcescale2[j]);
    }
    fprintf(output_file, "];\n\n");
  }


  //
  // print joint position data
  //
  // only in verbose logging mode, because the controller data also contains
  // this information.

  if (verbose) {
    fprintf(output_file, "data_joint_positions.t = [");
    for (int j=0; j<count_joint_positions; j++) {
      fprintf(output_file,"%f, ",data_joint_positions.t[j]);
    }
    fprintf(output_file, "];\n\n");

    fprintf(output_file, "data_joint_positions.name = {");
    for (int j=0; j<count_joint_positions; j++) {
      fprintf(output_file,"'%s', ",data_joint_positions.name[j].c_str());
    }
    fprintf(output_file, "};\n\n");

    fprintf(output_file, "data_joint_positions.q = [");
    for (int i=0;i<7;i++) {
      for (int j=0; j<count_joint_positions; j++) {
        fprintf(output_file,"%f, ",data_joint_positions.data[i][j]);
      }
      fprintf(output_file,"; ");
    }
    fprintf(output_file, "];\n\n");
  }

	fclose(output_file);

  //
  // print controller data, robot 0
  //

	sprintf(fname,"%s/data/Subject%02d/Set%02d_Trial%02d_controller_0.m",pathname.c_str(),subjectNumber,setNumber,trialNumber);
	output_file = fopen(fname,"a");
  if(output_file==NULL) {
    ROS_ERROR("Logger: Can't open file %s.",fname);
  }

	fprintf(output_file, "data_controller_0.t = [");
  for (int j=0; j<count_controller_data_0; j++) {
    fprintf(output_file,"%f, ",data_controller_0.t[j]);
  }
  fprintf(output_file, "];\n\n");

  if (verbose) {
    // redundant, because it's equal to diff(t)
    fprintf(output_file, "data_controller_0.dt = [");
    for (int j=0; j<count_controller_data_0; j++) {
      fprintf(output_file,"%f, ",data_controller_0.deltat[j]);
    }
    fprintf(output_file, "];\n\n");

    fprintf(output_file, "data_controller_0.controllerNumber = [");
    for (int j=0; j<count_controller_data_0; j++) {
      fprintf(output_file,"%d, ",data_controller_0.controllerNumber[j]);
    }
    fprintf(output_file, "];\n\n");

    fprintf(output_file, "data_controller_0.errorangle = [");
    for (int j=0; j<count_controller_data_0; j++) {
      fprintf(output_file,"%f, ",data_controller_0.errorangle[j]);
    }
    fprintf(output_file, "];\n\n");
  }

	fprintf(output_file, "data_controller_0.mass = [");
  for (int j=0; j<count_controller_data_0; j++) {
    fprintf(output_file,"%f, ",data_controller_0.mass[j]);
  }
  fprintf(output_file, "];\n\n");

  if (verbose) {
    fprintf(output_file, "data_controller_0.q = [");
    for (int i=0;i<7;i++) {
      for (int j=0; j<count_controller_data_0; j++) {
        fprintf(output_file,"%f, ",data_controller_0.position[i][j]);
      }
      fprintf(output_file,"; ");
    }
    fprintf(output_file, "];\n\n");

    fprintf(output_file, "data_controller_0.qs = [");
    for (int i=0;i<7;i++) {
      for (int j=0; j<count_controller_data_0; j++) {
        fprintf(output_file,"%f, ",data_controller_0.setpointRn[i][j]);
      }
      fprintf(output_file,"; ");
    }
    fprintf(output_file, "];\n\n");

    fprintf(output_file, "data_controller_0.qds = [");
    for (int i=0;i<7;i++) {
      for (int j=0; j<count_controller_data_0; j++) {
        fprintf(output_file,"%f, ",data_controller_0.setpointRnvelocity[i][j]);
      }
      fprintf(output_file,"; ");
    }
    fprintf(output_file, "];\n\n");

    fprintf(output_file, "data_controller_0.setpoint_null = [");
    for (int i=0;i<7;i++) {
      for (int j=0; j<count_controller_data_0; j++) {
        fprintf(output_file,"%f, ",data_controller_0.setpoint_null[i][j]);
      }
      fprintf(output_file,"; ");
    }
    fprintf(output_file, "];\n\n");

    fprintf(output_file, "data_controller_0.qd = [");
    for (int i=0;i<7;i++) {
      for (int j=0; j<count_controller_data_0; j++) {
        fprintf(output_file,"%f, ",data_controller_0.velocity[i][j]);
      }
      fprintf(output_file,"; ");
    }
    fprintf(output_file, "];\n\n");
  }

  fprintf(output_file, "data_controller_0.tau_gc = [");
  for (int i=0;i<7;i++) {
    for (int j=0; j<count_controller_data_0; j++) {
      fprintf(output_file,"%f, ",data_controller_0.torque_gc[i][j]);
    }
    fprintf(output_file,"; ");
  }
  fprintf(output_file, "];\n\n");

  if (verbose) {
    fprintf(output_file, "data_controller_0.tau_ct = [");
    for (int i=0;i<7;i++) {
      for (int j=0; j<count_controller_data_0; j++) {
        fprintf(output_file,"%f, ",data_controller_0.torque_ct[i][j]);
      }
      fprintf(output_file,"; ");
    }
    fprintf(output_file, "];\n\n");

    fprintf(output_file, "data_controller_0.tau_imp = [");
    for (int i=0;i<7;i++) {
      for (int j=0; j<count_controller_data_0; j++) {
        fprintf(output_file,"%f, ",data_controller_0.torque_imp[i][j]);
      }
      fprintf(output_file,"; ");
    }
    fprintf(output_file, "];\n\n");

    fprintf(output_file, "data_controller_0.tau_imp_null = [");
    for (int i=0;i<7;i++) {
      for (int j=0; j<count_controller_data_0; j++) {
        fprintf(output_file,"%f, ",data_controller_0.torque_imp_null[i][j]);
      }
      fprintf(output_file,"; ");
    }
    fprintf(output_file, "];\n\n");
  }

	fprintf(output_file, "data_controller_0.tau_imp_null_fric = [");
  for (int i=0;i<7;i++) {
    for (int j=0; j<count_controller_data_0; j++) {
      fprintf(output_file,"%f, ",data_controller_0.torque_imp_null_fric[i][j]);
    }
    fprintf(output_file,"; ");
  }
  fprintf(output_file, "];\n\n");

	fprintf(output_file, "data_controller_0.tau_external = [");
  for (int i=0;i<7;i++) {
    for (int j=0; j<count_controller_data_0; j++) {
      fprintf(output_file,"%f, ",data_controller_0.torque_external[i][j]);
    }
    fprintf(output_file,"; ");
  }
  fprintf(output_file, "];\n\n");

	fprintf(output_file, "data_controller_0.tau = [");
  for (int i=0;i<7;i++) {
    for (int j=0; j<count_controller_data_0; j++) {
      fprintf(output_file,"%f, ",data_controller_0.torque[i][j]);
    }
    fprintf(output_file,"; ");
  }
  fprintf(output_file, "];\n\n");

  if (verbose) {
    fprintf(output_file, "data_controller_0.xds = [");
    for (int i=0;i<6;i++) {
      for (int j=0; j<count_controller_data_0; j++) {
        fprintf(output_file,"%f, ",data_controller_0.setpointSE3velocity[i][j]);
      }
      fprintf(output_file,"; ");
    }
    fprintf(output_file, "];\n\n");

    fprintf(output_file, "data_controller_0.Kp_SE3 = [");
    for (int i=0;i<6;i++) {
      for (int j=0; j<count_controller_data_0; j++) {
        fprintf(output_file,"%f, ",data_controller_0.Kp_SE3[i][j]);
      }
      fprintf(output_file,"; ");
    }
    fprintf(output_file, "];\n\n");

    fprintf(output_file, "data_controller_0.Kd_SE3 = [");
    for (int i=0;i<6;i++) {
      for (int j=0; j<count_controller_data_0; j++) {
        fprintf(output_file,"%f, ",data_controller_0.Kd_SE3[i][j]);
      }
      fprintf(output_file,"; ");
    }
    fprintf(output_file, "];\n\n");

    fprintf(output_file, "data_controller_0.Kp_Rn = [");
    for (int i=0;i<7;i++) {
      for (int j=0; j<count_controller_data_0; j++) {
        fprintf(output_file,"%f, ",data_controller_0.Kp_Rn[i][j]);
      }
      fprintf(output_file,"; ");
    }
    fprintf(output_file, "];\n\n");

    fprintf(output_file, "data_controller_0.Kd_Rn = [");
    for (int i=0;i<7;i++) {
      for (int j=0; j<count_controller_data_0; j++) {
        fprintf(output_file,"%f, ",data_controller_0.Kd_Rn[i][j]);
      }
      fprintf(output_file,"; ");
    }
    fprintf(output_file, "];\n\n");
  }

	fprintf(output_file, "k = [%f];\n",data_controller_0.Kp_SE3[1][0]);
	fprintf(output_file, "b = [%f];\n",data_controller_0.Kd_SE3[1][0]);

  if (verbose) {
    fprintf(output_file, "data_controller_0.erroraxis = [");
    for (int i=0;i<3;i++) {
      for (int j=0; j<count_controller_data_0; j++) {
        fprintf(output_file,"%f, ",data_controller_0.erroraxis[i][j]);
      }
      fprintf(output_file,"; ");
    }
    fprintf(output_file, "];\n\n");

    fprintf(output_file, "data_controller_0.M = zeros(3,3,%d);\n", count_controller_data_0);
    for (int i=0;i<count_controller_data_0;i++) {
      fprintf(output_file, "data_controller_0.M(:,:,%d) = [",i+1);
      for (int j=0;j<3;j++) {
        for (int k=0;k<3;k++) {
          fprintf(output_file,"%f, ",data_controller_0.M[j][k][i]);
        }
        fprintf(output_file,"; ");
      }
      fprintf(output_file,"];\n");
    }
    fprintf(output_file,"\n");

    fprintf(output_file, "data_controller_0.M_OS = zeros(6,6,%d);\n", count_controller_data_0);
    for (int i=0;i<count_controller_data_0;i++) {
      fprintf(output_file, "data_controller_0.M_OS(:,:,%d) = [",i+1);
      for (int j=0;j<6;j++) {
        for (int k=0;k<6;k++) {
          fprintf(output_file,"%f, ",data_controller_0.M_OS[j][k][i]);
        }
        fprintf(output_file,"; ");
      }
      fprintf(output_file,"];\n");
    }
    fprintf(output_file,"\n");

    fprintf(output_file, "data_controller_0.M_JS = zeros(7,7,%d);\n", count_controller_data_0);
    for (int i=0;i<count_controller_data_0;i++) {
      fprintf(output_file, "data_controller_0.M_JS(:,:,%d) = [",i+1);
      for (int j=0;j<7;j++) {
        for (int k=0;k<7;k++) {
          fprintf(output_file,"%f, ",data_controller_0.M_JS[j][k][i]);
        }
        fprintf(output_file,"; ");
      }
      fprintf(output_file,"];\n");
    }
    fprintf(output_file,"\n");

    fprintf(output_file, "data_controller_0.pose = zeros(4,4,%d);\n", count_controller_data_0);
    for (int i=0;i<count_controller_data_0;i++) {
      fprintf(output_file, "data_controller_0.pose(:,:,%d) = [",i+1);
      for (int j=0;j<4;j++) {
        for (int k=0;k<4;k++) {
          fprintf(output_file,"%f, ",data_controller_0.pose[j][k][i]);
        }
        fprintf(output_file,"; ");
      }
      fprintf(output_file,"];\n");
    }
    fprintf(output_file,"\n");

    fprintf(output_file, "data_controller_0.setpointSE3 = zeros(4,4,%d);\n", count_controller_data_0);
    for (int i=0;i<count_controller_data_0;i++) {
      fprintf(output_file, "data_controller_0.setpointSE3(:,:,%d) = [",i+1);
      for (int j=0;j<4;j++) {
        for (int k=0;k<4;k++) {
          fprintf(output_file,"%f, ",data_controller_0.setpointSE3[j][k][i]);
        }
        fprintf(output_file,"; ");
      }
      fprintf(output_file,"];\n");
    }
    fprintf(output_file,"\n");
  }

	fprintf(output_file, "data_controller_0.x_a = [");
  for (int j=0; j<count_controller_data_0; j++) {
    fprintf(output_file,"%f, ",data_controller_0.pose[1][3][j]);
  }
  fprintf(output_file, "];\n\n");

	fprintf(output_file, "data_controller_0.x_d1 = [");
  for (int j=0; j<count_controller_data_0; j++) {
    fprintf(output_file,"%f, ",data_controller_0.setpointSE3[1][3][j]);
  }
  fprintf(output_file, "];\n\n");

	fprintf(output_file, "data_controller_0.xd_d1 = [");
  for (int j=0; j<count_controller_data_0; j++) {
    fprintf(output_file,"%f, ",data_controller_0.setpointSE3velocity[1][j]);
  }
  fprintf(output_file, "];\n\n");

  if (verbose) {
    fprintf(output_file, "data_controller_0.Kp_null = [");
    for (int j=0; j<count_controller_data_0; j++) {
      fprintf(output_file,"%f, ",data_controller_0.Kp_null[j]);
    }
    fprintf(output_file, "];\n\n");

    fprintf(output_file, "data_controller_0.Kd_null = [");
    for (int j=0; j<count_controller_data_0; j++) {
      fprintf(output_file,"%f, ",data_controller_0.Kd_null[j]);
    }
    fprintf(output_file, "];\n\n");

    fprintf(output_file, "data_controller_0.friction_gain = [");
    for (int j=0; j<count_controller_data_0; j++) {
      fprintf(output_file,"%f, ",data_controller_0.friction_gain[j]);
    }
    fprintf(output_file, "];\n\n");

    fprintf(output_file, "data_controller_0.impulse = [");
    for (int j=0; j<count_controller_data_0; j++) {
      fprintf(output_file,"%f, ",data_controller_0.impulse[j]);
    }
    fprintf(output_file, "];\n\n");
  }

	fprintf(output_file, "data_controller_0.forceExternal = [");
  for (int j=0; j<count_controller_data_0; j++) {
    fprintf(output_file,"%f, ",data_controller_0.forceExternal[j]);
  }
  fprintf(output_file, "];\n\n");

	fprintf(output_file, "data_controller_0.tracking_goal = [");
  for (int j=0; j<count_controller_data_0; j++) {
    fprintf(output_file,"%f, ",data_controller_0.tracking_goal[j]);
  }
  fprintf(output_file, "];\n\n");


	fclose(output_file);

  //
  // print controller data, robot 1
  //
  // this is only relevant to the forcemin task.

  if (verbose || (data_trial_info.task == 1)) {

    sprintf(fname,"%s/data/Subject%02d/Set%02d_Trial%02d_controller_1.m",pathname.c_str(),subjectNumber,setNumber,trialNumber);
    output_file = fopen(fname,"a");
    if(output_file==NULL) {
      ROS_ERROR("Logger: Can't open file %s.",fname);
    }

    fprintf(output_file, "data_controller_1.t = [");
    for (int j=0; j<count_controller_data_1; j++) {
      fprintf(output_file,"%f, ",data_controller_1.t[j]);
    }
    fprintf(output_file, "];\n\n");

    if (verbose) {
      // redundant, because it's equal to diff(t)
      fprintf(output_file, "data_controller_1.dt = [");
      for (int j=0; j<count_controller_data_1; j++) {
        fprintf(output_file,"%f, ",data_controller_1.deltat[j]);
      }
      fprintf(output_file, "];\n\n");

      fprintf(output_file, "data_controller_1.controllerNumber = [");
      for (int j=0; j<count_controller_data_1; j++) {
        fprintf(output_file,"%d, ",data_controller_1.controllerNumber[j]);
      }
      fprintf(output_file, "];\n\n");

      fprintf(output_file, "data_controller_1.errorangle = [");
      for (int j=0; j<count_controller_data_1; j++) {
        fprintf(output_file,"%f, ",data_controller_1.errorangle[j]);
      }
      fprintf(output_file, "];\n\n");
    }

    fprintf(output_file, "data_controller_1.mass = [");
    for (int j=0; j<count_controller_data_1; j++) {
      fprintf(output_file,"%f, ",data_controller_1.mass[j]);
    }
    fprintf(output_file, "];\n\n");

    if (verbose) {
      fprintf(output_file, "data_controller_1.q = [");
      for (int i=0;i<7;i++) {
        for (int j=0; j<count_controller_data_1; j++) {
          fprintf(output_file,"%f, ",data_controller_1.position[i][j]);
        }
        fprintf(output_file,"; ");
      }
      fprintf(output_file, "];\n\n");

      fprintf(output_file, "data_controller_1.qs = [");
      for (int i=0;i<7;i++) {
        for (int j=0; j<count_controller_data_1; j++) {
          fprintf(output_file,"%f, ",data_controller_1.setpointRn[i][j]);
        }
        fprintf(output_file,"; ");
      }
      fprintf(output_file, "];\n\n");

      fprintf(output_file, "data_controller_1.qds = [");
      for (int i=0;i<7;i++) {
        for (int j=0; j<count_controller_data_1; j++) {
          fprintf(output_file,"%f, ",data_controller_1.setpointRnvelocity[i][j]);
        }
        fprintf(output_file,"; ");
      }
      fprintf(output_file, "];\n\n");

      fprintf(output_file, "data_controller_1.setpoint_null = [");
      for (int i=0;i<7;i++) {
        for (int j=0; j<count_controller_data_1; j++) {
          fprintf(output_file,"%f, ",data_controller_1.setpoint_null[i][j]);
        }
        fprintf(output_file,"; ");
      }
      fprintf(output_file, "];\n\n");

      fprintf(output_file, "data_controller_1.qd = [");
      for (int i=0;i<7;i++) {
        for (int j=0; j<count_controller_data_1; j++) {
          fprintf(output_file,"%f, ",data_controller_1.velocity[i][j]);
        }
        fprintf(output_file,"; ");
      }
      fprintf(output_file, "];\n\n");
    }

    fprintf(output_file, "data_controller_1.tau_gc = [");
    for (int i=0;i<7;i++) {
      for (int j=0; j<count_controller_data_1; j++) {
        fprintf(output_file,"%f, ",data_controller_1.torque_gc[i][j]);
      }
      fprintf(output_file,"; ");
    }
    fprintf(output_file, "];\n\n");

    if (verbose) {
      fprintf(output_file, "data_controller_1.tau_ct = [");
      for (int i=0;i<7;i++) {
        for (int j=0; j<count_controller_data_1; j++) {
          fprintf(output_file,"%f, ",data_controller_1.torque_ct[i][j]);
        }
        fprintf(output_file,"; ");
      }
      fprintf(output_file, "];\n\n");

      fprintf(output_file, "data_controller_1.tau_imp = [");
      for (int i=0;i<7;i++) {
        for (int j=0; j<count_controller_data_1; j++) {
          fprintf(output_file,"%f, ",data_controller_1.torque_imp[i][j]);
        }
        fprintf(output_file,"; ");
      }
      fprintf(output_file, "];\n\n");

      fprintf(output_file, "data_controller_1.tau_imp_null = [");
      for (int i=0;i<7;i++) {
        for (int j=0; j<count_controller_data_1; j++) {
          fprintf(output_file,"%f, ",data_controller_1.torque_imp_null[i][j]);
        }
        fprintf(output_file,"; ");
      }
      fprintf(output_file, "];\n\n");
    }

    fprintf(output_file, "data_controller_1.tau_imp_null_fric = [");
    for (int i=0;i<7;i++) {
      for (int j=0; j<count_controller_data_1; j++) {
        fprintf(output_file,"%f, ",data_controller_1.torque_imp_null_fric[i][j]);
      }
      fprintf(output_file,"; ");
    }
    fprintf(output_file, "];\n\n");

    fprintf(output_file, "data_controller_1.tau_external = [");
    for (int i=0;i<7;i++) {
      for (int j=0; j<count_controller_data_1; j++) {
        fprintf(output_file,"%f, ",data_controller_1.torque_external[i][j]);
      }
      fprintf(output_file,"; ");
    }
    fprintf(output_file, "];\n\n");

    fprintf(output_file, "data_controller_1.tau = [");
    for (int i=0;i<7;i++) {
      for (int j=0; j<count_controller_data_1; j++) {
        fprintf(output_file,"%f, ",data_controller_1.torque[i][j]);
      }
      fprintf(output_file,"; ");
    }
    fprintf(output_file, "];\n\n");

    if (verbose) {
      fprintf(output_file, "data_controller_1.xds = [");
      for (int i=0;i<6;i++) {
        for (int j=0; j<count_controller_data_1; j++) {
          fprintf(output_file,"%f, ",data_controller_1.setpointSE3velocity[i][j]);
        }
        fprintf(output_file,"; ");
      }
      fprintf(output_file, "];\n\n");

      fprintf(output_file, "data_controller_1.Kp_SE3 = [");
      for (int i=0;i<6;i++) {
        for (int j=0; j<count_controller_data_1; j++) {
          fprintf(output_file,"%f, ",data_controller_1.Kp_SE3[i][j]);
        }
        fprintf(output_file,"; ");
      }
      fprintf(output_file, "];\n\n");

      fprintf(output_file, "data_controller_1.Kd_SE3 = [");
      for (int i=0;i<6;i++) {
        for (int j=0; j<count_controller_data_1; j++) {
          fprintf(output_file,"%f, ",data_controller_1.Kd_SE3[i][j]);
        }
        fprintf(output_file,"; ");
      }
      fprintf(output_file, "];\n\n");

      fprintf(output_file, "data_controller_1.Kp_Rn = [");
      for (int i=0;i<7;i++) {
        for (int j=0; j<count_controller_data_1; j++) {
          fprintf(output_file,"%f, ",data_controller_1.Kp_Rn[i][j]);
        }
        fprintf(output_file,"; ");
      }
      fprintf(output_file, "];\n\n");

      fprintf(output_file, "data_controller_1.Kd_Rn = [");
      for (int i=0;i<7;i++) {
        for (int j=0; j<count_controller_data_1; j++) {
          fprintf(output_file,"%f, ",data_controller_1.Kd_Rn[i][j]);
        }
        fprintf(output_file,"; ");
      }
      fprintf(output_file, "];\n\n");
    }

    fprintf(output_file, "k2 = [%f];\n",data_controller_1.Kp_SE3[1][0]);
    fprintf(output_file, "b2 = [%f];\n",data_controller_1.Kd_SE3[1][0]);

    if (verbose) {
      fprintf(output_file, "data_controller_1.erroraxis = [");
      for (int i=0;i<3;i++) {
        for (int j=0; j<count_controller_data_1; j++) {
          fprintf(output_file,"%f, ",data_controller_1.erroraxis[i][j]);
        }
        fprintf(output_file,"; ");
      }
      fprintf(output_file, "];\n\n");

      fprintf(output_file, "data_controller_1.M = zeros(3,3,%d);\n", count_controller_data_1);
      for (int i=0;i<count_controller_data_1;i++) {
        fprintf(output_file, "data_controller_1.M(:,:,%d) = [",i+1);
        for (int j=0;j<3;j++) {
          for (int k=0;k<3;k++) {
            fprintf(output_file,"%f, ",data_controller_1.M[j][k][i]);
          }
          fprintf(output_file,"; ");
        }
        fprintf(output_file,"];\n");
      }
      fprintf(output_file,"\n");

      fprintf(output_file, "data_controller_1.M_OS = zeros(6,6,%d);\n", count_controller_data_1);
      for (int i=0;i<count_controller_data_1;i++) {
        fprintf(output_file, "data_controller_1.M_OS(:,:,%d) = [",i+1);
        for (int j=0;j<6;j++) {
          for (int k=0;k<6;k++) {
            fprintf(output_file,"%f, ",data_controller_1.M_OS[j][k][i]);
          }
          fprintf(output_file,"; ");
        }
        fprintf(output_file,"];\n");
      }
      fprintf(output_file,"\n");

      fprintf(output_file, "data_controller_1.M_JS = zeros(7,7,%d);\n", count_controller_data_1);
      for (int i=0;i<count_controller_data_1;i++) {
        fprintf(output_file, "data_controller_1.M_JS(:,:,%d) = [",i+1);
        for (int j=0;j<7;j++) {
          for (int k=0;k<7;k++) {
            fprintf(output_file,"%f, ",data_controller_1.M_JS[j][k][i]);
          }
          fprintf(output_file,"; ");
        }
        fprintf(output_file,"];\n");
      }
      fprintf(output_file,"\n");

      fprintf(output_file, "data_controller_1.pose = zeros(4,4,%d);\n", count_controller_data_1);
      for (int i=0;i<count_controller_data_1;i++) {
        fprintf(output_file, "data_controller_1.pose(:,:,%d) = [",i+1);
        for (int j=0;j<4;j++) {
          for (int k=0;k<4;k++) {
            fprintf(output_file,"%f, ",data_controller_1.pose[j][k][i]);
          }
          fprintf(output_file,"; ");
        }
        fprintf(output_file,"];\n");
      }
      fprintf(output_file,"\n");

      fprintf(output_file, "data_controller_1.setpointSE3 = zeros(4,4,%d);\n", count_controller_data_1);
      for (int i=0;i<count_controller_data_1;i++) {
        fprintf(output_file, "data_controller_1.setpointSE3(:,:,%d) = [",i+1);
        for (int j=0;j<4;j++) {
          for (int k=0;k<4;k++) {
            fprintf(output_file,"%f, ",data_controller_1.setpointSE3[j][k][i]);
          }
          fprintf(output_file,"; ");
        }
        fprintf(output_file,"];\n");
        }
      fprintf(output_file,"\n");
    }

    fprintf(output_file, "data_controller_1.x_a = [");
    for (int j=0; j<count_controller_data_1; j++) {
      fprintf(output_file,"%f, ",data_controller_1.pose[1][3][j]);
    }
    fprintf(output_file, "];\n\n");

    fprintf(output_file, "data_controller_1.x_d2 = [");
    for (int j=0; j<count_controller_data_1; j++) {
      fprintf(output_file,"%f, ",data_controller_1.setpointSE3[1][3][j]);
    }
    fprintf(output_file, "];\n\n");

    fprintf(output_file, "data_controller_1.xd_d2 = [");
    for (int j=0; j<count_controller_data_1; j++) {
      fprintf(output_file,"%f, ",data_controller_1.setpointSE3velocity[1][j]);
    }
    fprintf(output_file, "];\n\n");

    if (verbose) {
      fprintf(output_file, "data_controller_1.Kp_null = [");
      for (int j=0; j<count_controller_data_1; j++) {
        fprintf(output_file,"%f, ",data_controller_1.Kp_null[j]);
      }
      fprintf(output_file, "];\n\n");

      fprintf(output_file, "data_controller_1.Kd_null = [");
      for (int j=0; j<count_controller_data_1; j++) {
        fprintf(output_file,"%f, ",data_controller_1.Kd_null[j]);
      }
      fprintf(output_file, "];\n\n");

      fprintf(output_file, "data_controller_1.friction_gain = [");
      for (int j=0; j<count_controller_data_1; j++) {
        fprintf(output_file,"%f, ",data_controller_1.friction_gain[j]);
      }
      fprintf(output_file, "];\n\n");

      fprintf(output_file, "data_controller_1.impulse = [");
      for (int j=0; j<count_controller_data_1; j++) {
        fprintf(output_file,"%f, ",data_controller_1.impulse[j]);
      }
      fprintf(output_file, "];\n\n");
    }

    fprintf(output_file, "data_controller_1.forceExternal = [");
    for (int j=0; j<count_controller_data_1; j++) {
      fprintf(output_file,"%f, ",data_controller_1.forceExternal[j]);
    }
    fprintf(output_file, "];\n\n");

    fclose(output_file);

  }

  //
  // print trajectory data, robot 0
  //
  // only in verbose mode, because this stuff is all duplicated in controller_data.

  if (verbose) {
    sprintf(fname,"%s/data/Subject%02d/Set%02d_Trial%02d_trajectory_0.m",pathname.c_str(),subjectNumber,setNumber,trialNumber);
    output_file = fopen(fname,"a");
    if(output_file==NULL) {
      ROS_ERROR("Logger: Can't open file %s.",fname);
    }

    fprintf(output_file, "data_trajectory_0.t = [");
    for (int j=0; j<count_trajectory_0; j++) {
      fprintf(output_file,"%f, ",data_trajectory_0.t[j]);
    }
    fprintf(output_file, "];\n\n");

    fprintf(output_file, "data_trajectory_0.type = [");
    for (int j=0; j<count_trajectory_0; j++) {
      fprintf(output_file,"%d, ",data_trajectory_0.trajectory[j]);
    }
    fprintf(output_file, "];\n\n");

    fprintf(output_file, "data_trajectory_0.maxCommandSpeed = [");
    for (int j=0; j<count_trajectory_0; j++) {
      fprintf(output_file,"%d, ",(int)(data_trajectory_0.maxCommandSpeed[j]));
    }
    fprintf(output_file, "];\n\n");

    fprintf(output_file, "data_trajectory_0.qs = [");
    for (int i=0;i<7;i++) {
      for (int j=0; j<count_trajectory_0; j++) {
        fprintf(output_file,"%f, ",data_trajectory_0.qs[i][j]);
      }
      fprintf(output_file,"; ");
    }
    fprintf(output_file, "];\n\n");

    fprintf(output_file, "data_trajectory_0.vws = [");
    for (int i=0;i<6;i++) {
      for (int j=0; j<count_trajectory_0; j++) {
        fprintf(output_file,"%f, ",data_trajectory_0.vws[i][j]);
      }
      fprintf(output_file,"; ");
    }
    fprintf(output_file, "];\n\n");

    fprintf(output_file, "data_trajectory_0.Rts = zeros(4,4,%d);\n", count_trajectory_0);
    for (int i=0;i<count_trajectory_0;i++) {
      fprintf(output_file, "data_trajectory_0.Rts(:,:,%d) = [",i+1);
      for (int j=0;j<4;j++) {
        for (int k=0;k<4;k++) {
          fprintf(output_file,"%f, ",data_trajectory_0.Rts[j][k][i]);
        }
        fprintf(output_file,"; ");
      }
      fprintf(output_file,"];\n");
    }
    fprintf(output_file,"\n");

    fclose(output_file);
  }

  //
  // print trajectory data, robot 1
  //
  // only in verbose mode, because this stuff is duplicated in controller_data

  if (verbose) {
    sprintf(fname,"%s/data/Subject%02d/Set%02d_Trial%02d_trajectory_1.m",pathname.c_str(),subjectNumber,setNumber,trialNumber);
    output_file = fopen(fname,"a");
    if(output_file==NULL) {
      ROS_ERROR("Logger: Can't open file %s.",fname);
    }

    fprintf(output_file, "data_trajectory_1.t = [");
    for (int j=0; j<count_trajectory_1; j++) {
      fprintf(output_file,"%f, ",data_trajectory_1.t[j]);
    }
    fprintf(output_file, "];\n\n");

    fprintf(output_file, "data_trajectory_1.type = [");
    for (int j=0; j<count_trajectory_1; j++) {
      fprintf(output_file,"%d, ",data_trajectory_1.trajectory[j]);
    }
    fprintf(output_file, "];\n\n");

    fprintf(output_file, "data_trajectory_1.qs = [");
    for (int i=0;i<7;i++) {
      for (int j=0; j<count_trajectory_1; j++) {
        fprintf(output_file,"%f, ",data_trajectory_1.qs[i][j]);
      }
      fprintf(output_file,"; ");
    }
    fprintf(output_file, "];\n\n");

    fprintf(output_file, "data_trajectory_1.vws = [");
    for (int i=0;i<6;i++) {
      for (int j=0; j<count_trajectory_1; j++) {
        fprintf(output_file,"%f, ",data_trajectory_1.vws[i][j]);
      }
      fprintf(output_file,"; ");
    }
    fprintf(output_file, "];\n\n");

    fprintf(output_file, "data_trajectory_1.Rts = zeros(4,4,%d);\n", count_trajectory_1);
    for (int i=0;i<count_trajectory_1;i++) {
      fprintf(output_file, "data_trajectory_1.Rts(:,:,%d) = [",i+1);
      for (int j=0;j<4;j++) {
        for (int k=0;k<4;k++) {
          fprintf(output_file,"%f, ",data_trajectory_1.Rts[j][k][i]);
        }
        fprintf(output_file,"; ");
      }
      fprintf(output_file,"];\n");
    }
    fprintf(output_file,"\n");

    fclose(output_file);
  }


  /*
  // print state and feedback information
	// desired user position
	fprintf(output_file, "data.t = [");
  for (int j=0; j<count_data; j++) {
    fprintf(output_file,"%f, ",data.t[j]);
  }
  fprintf(output_file, "];\n\n");

	fprintf(output_file, "data.x_d1 = [");
  for (int j=0; j<count_data; j++) {
    fprintf(output_file,"%f, ",data.x_d1[j]);
  }
  fprintf(output_file, "];\n\n");

	// desired user velocity
	fprintf(output_file, "data.xd_d1 = [");
  for (int j=0; j<count_data; j++) {
    fprintf(output_file,"%f, ",data.xd_d1[j]);
  }
  fprintf(output_file, "];\n\n");

  // desired moving object position
	fprintf(output_file, "data.x_d2 = [");
  for (int j=0; j<count_data; j++) {
    fprintf(output_file,"%f, ",data.x_d2[j]);
  }
  fprintf(output_file, "];\n\n");

	// desired moving object velocity
	fprintf(output_file, "data.xd_d2 = [");
  for (int j=0; j<count_data; j++) {
    fprintf(output_file,"%f, ",data.xd_d2[j]);
  }
  fprintf(output_file, "];\n\n");

	// actual position
	fprintf(output_file, "data.x_a = [");
  for (int j=0; j<count_data; j++) {
    fprintf(output_file,"%f, ",data.x_a[j]);
  }
  fprintf(output_file, "];\n\n");

  // actual velocity
  fprintf(output_file, "data.xd_a = [");
  for (int j=0; j<count_data; j++) {
    fprintf(output_file,"%f, ",data.xd_a[j]);
  }
  fprintf(output_file, "];\n\n");

  // environment force
	fprintf(output_file, "data.Fenv = [");
  for (int j=0; j<count_data; j++) {
    fprintf(output_file,"%f, ",data.Fenv[j]);
  }
  fprintf(output_file, "];\n\n");

  if (data_trial_info.task == 0) {
    // environment force
    fprintf(output_file, "data.x_t = [");
    for (int j=0; j<count_data; j++) {
      fprintf(output_file,"%f, ",data.x_t[j]);
    }
    fprintf(output_file, "];\n\n");
  }
  */


  /*
  // projectile info
  if (data_trial_info.task == 0) {

    // time stamps
    fprintf(output_file, "data_projectiles.t = [");
    for (int j=0; j<count_projectiles; j++) {
      fprintf(output_file,"%f, ",data_projectiles.t[j]);
    }
    fprintf(output_file, "];\n\n");
    
    // projectile positions
    fprintf(output_file, "data_projectiles.x_p = [");
    for (int i=0;i<5;i++) {
      for (int j=0; j<count_projectiles; j++) {
        fprintf(output_file,"%f, ",data_projectiles.x_p[i][j]);
      }
      fprintf(output_file, "; ");
    }
    fprintf(output_file, "];\n\n");

    // impulses
    fprintf(output_file, "data_projectiles.impulse = [");
    for (int j=0; j<count_projectiles; j++) {
      fprintf(output_file,"%f, ",data_projectiles.impulse[j]);
    }
    fprintf(output_file, "];\n\n");

  }
	fclose(output_file);
  */


}


void ClearArrays(void) {
  ROS_INFO("Logger: Clearing data arrays...");

	memset(data_gains.t,0,MAX_SAMPLES*sizeof(double));
	memset(data_gains.k1,0,MAX_SAMPLES*sizeof(double));
	memset(data_gains.b1,0,MAX_SAMPLES*sizeof(double));
	memset(data_gains.m1,0,MAX_SAMPLES*sizeof(double));
	memset(data_gains.k2,0,MAX_SAMPLES*sizeof(double));
	memset(data_gains.b2,0,MAX_SAMPLES*sizeof(double));
	memset(data_gains.m2,0,MAX_SAMPLES*sizeof(double));

  /*
	memset(data_projectiles.t,0,MAX_SAMPLES*sizeof(double));
	memset(data_projectiles.x_p,0,MAX_SAMPLES*5*sizeof(double));
	memset(data_projectiles.impulse,0,MAX_SAMPLES*sizeof(double));
  */

	memset(data_input.t,0,MAX_SAMPLES*sizeof(double));
	memset(data_input.Fu_counts,0,MAX_SAMPLES*sizeof(double));
	memset(data_input.Fu_raw,0,MAX_SAMPLES*sizeof(double));
	memset(data_input.Fu,0,MAX_SAMPLES*sizeof(double));
	memset(data_input.Fenv_counts,0,MAX_SAMPLES*sizeof(double));
	memset(data_input.Fenv_raw,0,MAX_SAMPLES*sizeof(double));
	memset(data_input.Fenv,0,MAX_SAMPLES*sizeof(double));
	memset(data_input.pot1_counts,0,MAX_SAMPLES*sizeof(double));
	memset(data_input.pot1_raw,0,MAX_SAMPLES*sizeof(double));
	memset(data_input.pot1,0,MAX_SAMPLES*sizeof(double));
	memset(data_input.pot2_counts,0,MAX_SAMPLES*sizeof(double));
	memset(data_input.pot2_raw,0,MAX_SAMPLES*sizeof(double));
	memset(data_input.pot2,0,MAX_SAMPLES*sizeof(double));
	memset(data_input.zeroforce1,0,MAX_SAMPLES*sizeof(double));
	memset(data_input.zeroforce2,0,MAX_SAMPLES*sizeof(double));
	memset(data_input.forcescale1,0,MAX_SAMPLES*sizeof(double));
	memset(data_input.forcescale2,0,MAX_SAMPLES*sizeof(double));

	memset(data_joint_positions.t,0,MAX_SAMPLES*sizeof(double));
	memset(data_joint_positions.data,0,7*MAX_SAMPLES*sizeof(double));

	memset(data_controller_0.t,0,MAX_SAMPLES*sizeof(double));
	memset(data_controller_0.deltat,0,MAX_SAMPLES*sizeof(double));
	memset(data_controller_0.position,0,7*MAX_SAMPLES*sizeof(double));
	memset(data_controller_0.velocity,0,7*MAX_SAMPLES*sizeof(double));
	memset(data_controller_0.torque_gc,0,7*MAX_SAMPLES*sizeof(double));
	memset(data_controller_0.torque_ct,0,7*MAX_SAMPLES*sizeof(double));
	memset(data_controller_0.torque_imp,0,7*MAX_SAMPLES*sizeof(double));
	memset(data_controller_0.torque_imp_null,0,7*MAX_SAMPLES*sizeof(double));
	memset(data_controller_0.torque_imp_null_fric,0,7*MAX_SAMPLES*sizeof(double));
	memset(data_controller_0.torque_external,0,7*MAX_SAMPLES*sizeof(double));
	memset(data_controller_0.torque,0,7*MAX_SAMPLES*sizeof(double));
	memset(data_controller_0.setpointRn,0,7*MAX_SAMPLES*sizeof(double));
	memset(data_controller_0.setpointRnvelocity,0,7*MAX_SAMPLES*sizeof(double));
	memset(data_controller_0.setpointSE3velocity,0,6*MAX_SAMPLES*sizeof(double));
	memset(data_controller_0.Kp_SE3,0,6*MAX_SAMPLES*sizeof(double));
	memset(data_controller_0.Kd_SE3,0,6*MAX_SAMPLES*sizeof(double));
	memset(data_controller_0.Kp_Rn,0,7*MAX_SAMPLES*sizeof(double));
	memset(data_controller_0.Kd_Rn,0,7*MAX_SAMPLES*sizeof(double));
	memset(data_controller_0.erroraxis,0,3*MAX_SAMPLES*sizeof(double));
	memset(data_controller_0.Kp_null,0,MAX_SAMPLES*sizeof(double));
	memset(data_controller_0.Kd_null,0,MAX_SAMPLES*sizeof(double));
	memset(data_controller_0.friction_gain,0,MAX_SAMPLES*sizeof(double));
	memset(data_controller_0.impulse,0,MAX_SAMPLES*sizeof(double));
	memset(data_controller_0.forceExternal,0,MAX_SAMPLES*sizeof(double));
	memset(data_controller_0.pose,0,4*4*MAX_SAMPLES*sizeof(double));
	memset(data_controller_0.M,0,3*3*MAX_SAMPLES*sizeof(double));
	memset(data_controller_0.M_OS,0,6*6*MAX_SAMPLES*sizeof(double));
	memset(data_controller_0.M_JS,0,7*7*MAX_SAMPLES*sizeof(double));
	memset(data_controller_0.setpointSE3,0,4*4*MAX_SAMPLES*sizeof(double));
	memset(data_controller_0.controllerNumber,0,MAX_SAMPLES*sizeof(int));
	memset(data_controller_0.errorangle,0,MAX_SAMPLES*sizeof(double));
	memset(data_controller_0.mass,0,MAX_SAMPLES*sizeof(double));
	memset(data_controller_0.tracking_goal,0,MAX_SAMPLES*sizeof(double));

	memset(data_controller_1.t,0,MAX_SAMPLES*sizeof(double));
	memset(data_controller_1.deltat,0,MAX_SAMPLES*sizeof(double));
	memset(data_controller_1.position,0,7*MAX_SAMPLES*sizeof(double));
	memset(data_controller_1.velocity,0,7*MAX_SAMPLES*sizeof(double));
	memset(data_controller_1.torque_gc,0,7*MAX_SAMPLES*sizeof(double));
	memset(data_controller_1.torque_ct,0,7*MAX_SAMPLES*sizeof(double));
	memset(data_controller_1.torque_imp,0,7*MAX_SAMPLES*sizeof(double));
	memset(data_controller_1.torque_imp_null,0,7*MAX_SAMPLES*sizeof(double));
	memset(data_controller_1.torque_imp_null_fric,0,7*MAX_SAMPLES*sizeof(double));
	memset(data_controller_1.torque_external,0,7*MAX_SAMPLES*sizeof(double));
	memset(data_controller_1.torque,0,7*MAX_SAMPLES*sizeof(double));
	memset(data_controller_1.setpointRn,0,7*MAX_SAMPLES*sizeof(double));
	memset(data_controller_1.setpointRnvelocity,0,7*MAX_SAMPLES*sizeof(double));
	memset(data_controller_1.setpointSE3velocity,0,6*MAX_SAMPLES*sizeof(double));
	memset(data_controller_1.Kp_SE3,0,6*MAX_SAMPLES*sizeof(double));
	memset(data_controller_1.Kd_SE3,0,6*MAX_SAMPLES*sizeof(double));
	memset(data_controller_1.Kp_Rn,0,7*MAX_SAMPLES*sizeof(double));
	memset(data_controller_1.Kd_Rn,0,7*MAX_SAMPLES*sizeof(double));
	memset(data_controller_1.erroraxis,0,3*MAX_SAMPLES*sizeof(double));
	memset(data_controller_1.Kp_null,0,MAX_SAMPLES*sizeof(double));
	memset(data_controller_1.Kd_null,0,MAX_SAMPLES*sizeof(double));
	memset(data_controller_1.friction_gain,0,MAX_SAMPLES*sizeof(double));
	memset(data_controller_1.impulse,0,MAX_SAMPLES*sizeof(double));
	memset(data_controller_1.forceExternal,0,MAX_SAMPLES*sizeof(double));
	memset(data_controller_1.pose,0,4*4*MAX_SAMPLES*sizeof(double));
	memset(data_controller_1.M,0,3*3*MAX_SAMPLES*sizeof(double));
	memset(data_controller_1.M_OS,0,6*6*MAX_SAMPLES*sizeof(double));
	memset(data_controller_1.M_JS,0,7*7*MAX_SAMPLES*sizeof(double));
	memset(data_controller_1.setpointSE3,0,4*4*MAX_SAMPLES*sizeof(double));
	memset(data_controller_1.controllerNumber,0,MAX_SAMPLES*sizeof(int));
	memset(data_controller_1.errorangle,0,MAX_SAMPLES*sizeof(double));
	memset(data_controller_1.mass,0,MAX_SAMPLES*sizeof(double));

	memset(data_trajectory_0.t,0,MAX_SAMPLES*sizeof(double));
	memset(data_trajectory_0.trajectory,0,MAX_SAMPLES*sizeof(int));
	memset(data_trajectory_0.qs,0,7*MAX_SAMPLES*sizeof(double));
	memset(data_trajectory_0.vws,0,6*MAX_SAMPLES*sizeof(double));
	memset(data_trajectory_0.Rts,0,4*4*MAX_SAMPLES*sizeof(double));

	memset(data_trajectory_1.t,0,MAX_SAMPLES*sizeof(double));
	memset(data_trajectory_1.trajectory,0,MAX_SAMPLES*sizeof(int));
	memset(data_trajectory_1.qs,0,7*MAX_SAMPLES*sizeof(double));
	memset(data_trajectory_1.vws,0,6*MAX_SAMPLES*sizeof(double));
	memset(data_trajectory_1.Rts,0,4*4*MAX_SAMPLES*sizeof(double));

  /*
	memset(data.t,0,MAX_SAMPLES*sizeof(double));
	memset(data.x_a,0,MAX_SAMPLES*sizeof(double));
	memset(data.x_d1,0,MAX_SAMPLES*sizeof(double));
	memset(data.x_d2,0,MAX_SAMPLES*sizeof(double));
	memset(data.xd_a,0,MAX_SAMPLES*sizeof(double));
	memset(data.xd_d1,0,MAX_SAMPLES*sizeof(double));
	memset(data.xd_d2,0,MAX_SAMPLES*sizeof(double));
	memset(data.x_t,0,MAX_SAMPLES*sizeof(double));
	memset(data.Fenv,0,MAX_SAMPLES*sizeof(double));
  */

  data_trial_info.subjectNumber = -1;
  data_trial_info.trialNumber = 0;
  data_trial_info.setNumber = 0;
  data_trial_info.set_type = -1;
  data_trial_info.task = -1;
  data_trial_info.admittance = 0;
  data_trial_info.deadband = 0;
  data_trial_info.direction = 0;

  ROS_INFO("Logger: Done.");
}


bool startLogging(wam_impedance_experiment::StartLogging::Request  &req,
                  wam_impedance_experiment::StartLogging::Response &res )
{
  data_trial_info.subjectNumber = req.subjectNumber;
  data_trial_info.trialNumber = req.trialNumber;
  data_trial_info.setNumber = req.setNumber;
  data_trial_info.set_type = req.set_type;           // practice (0), experiment (1), or training (2)
  data_trial_info.task = req.task;                   // forcemin (1) or tracking (0)
  data_trial_info.admittance = req.admittance;
  data_trial_info.deadband = req.deadband;
  data_trial_info.direction = req.direction;   // direction of sine wave

  count_data = 0;
  count_input = 0;
  count_projectiles = 0;
  count_gains = 0;
  count_joint_positions = 0;
  count_controller_data_0 = 0;
  count_controller_data_1 = 0;
  count_trajectory_0 = 0;
  count_trajectory_1 = 0;
  recordData = true;

  //ROS_INFO("Logger: Data logging started.");
  return true;
}

bool stopLogging(std_srvs::Empty::Request  &req,
                 std_srvs::Empty::Response &res )
{
  recordData = false;
  ROS_INFO("Logger: Data logging stopped.");

  return true;
}

bool writeData(wam_impedance_experiment::Int::Request  &req,
               wam_impedance_experiment::Int::Response &res )
{
  ROS_INFO("Logger: Writing data file.");
  WriteDataFile(data_trial_info.subjectNumber,data_trial_info.setNumber,data_trial_info.trialNumber,req.data);
  ClearArrays();

  return true;
}

int main(int argc, char **argv) {

  // set up ROS stuff
  ros::init(argc, argv, "logger");
  ros::NodeHandle n;
  ros::Rate loop_rate(1000); // loop rate in Hz


  // subscribers for receiving all data to be logged

  // user and moving object gains
  //ros::Subscriber sub_gains = n.subscribe<wam_impedance_experiment::Gains>("gains", 1000, GainsCallback);

  // daq input (user commands)
  ros::Subscriber sub_user_input = n.subscribe<wam_impedance_experiment::DAQinput>("daq_input", 1000, UserInputCallback);

  // robot joint angles
  ros::Subscriber sub_joint_angles = n.subscribe<sensor_msgs::JointState>("joint_states", 1000, JointStateCallback);

  // data from controller (joint torques, and the stuff used to calculate them)
  ros::Subscriber sub_controller_data = n.subscribe<wam_impedance_experiment::ControllerData>("controller_data", 1000, ControllerDataCallback);

  // commanded trajectory
  ros::Subscriber sub_trajectory = n.subscribe<wam_impedance_experiment::TrajectoryInfo>("trajectory", 1000, TrajectoryCallback);

  // experiment state info
  //ros::Subscriber sub_expt_state = n.subscribe<wam_impedance_experiment::ExperimentState>("experiment_state", 1000,ExperimentStateCallback);

  // projectile info
  //ros::Subscriber sub_projectiles = n.subscribe<wam_impedance_experiment::ProjectileInfo>("projectiles", 1000,ProjectilesCallback);

  // subscriber for activating and deactivating verbose logging
  ros::Subscriber sub_toggle_verbose_logging = n.subscribe<std_msgs::Empty>("toggle_verbose_logging", 1000, toggleVerboseLogging);


  // subscriber for receiving quit command
  ros::Subscriber sub_quit = n.subscribe<std_msgs::Empty>("quit", 1000, Quit);

  // services for starting and stopping data logging (should write data file on stop)
  ros::ServiceServer server_start_logging = n.advertiseService("start_logging", startLogging);
  ros::ServiceServer server_stop_logging = n.advertiseService("stop_logging", stopLogging);
  ros::ServiceServer server_write_data = n.advertiseService("write_data", writeData);


  // get path for storing data files
  pathname = ros::package::getPath("wam_impedance_experiment");


  while (ros::ok()) {

    ros::spinOnce();
    loop_rate.sleep();

  }

 
  return 0;
}
