#include "ros/ros.h"
#include <gsl/gsl_vector.h>
#include <gsl/gsl_matrix.h>
#include <gsl/gsl_blas.h>
#include <gsl/gsl_errno.h>
#include <gsl/gsl_linalg.h>
#include "simulation_impedance_experiment/ExperimentState.h"
#include "simulation_impedance_experiment/TaskType.h"
#include "simulation_impedance_experiment/SetType.h"
#include "simulation_impedance_experiment/IntegerRequest.h"
#include "simulation_impedance_experiment/Gains.h"
#include "simulation_impedance_experiment/DAQinput.h"
#include "simulation_impedance_experiment/DAQoutput.h"
#include "simulation_impedance_experiment/StartSimulation.h"
#include "std_srvs/Empty.h"
#include "std_msgs/Bool.h"

// structure to hold impedance info
typedef struct {
	double k;
	double b;
	double m;
} impedance;

// gains for user and moving object
impedance imp1;
impedance imp2;

// admittance and deadband for user input
const double admittance = 0.2;
const double deadband = 0.05;

// initial state of the system at the beginning of each trial is zero
ros::Time* t_start;
taskType task = forcemin;
setType set_type = training;
double x_a = 0.0;
double x_d1 = 0.0;
double x_d2 = 0.0;
double xd_a = 0.0;
double xd_d1 = 0.0;
double xd_d2 = 0.0;
double x_a_old = 0.0;
double x_d1_old = 0.0;
double x_d2_old = 0.0;
double xd_a_old = 0.0;
double xd_d1_old = 0.0;
double xd_d2_old = 0.0;
double Fu_raw = 0.0;
double Fu = 0.0;
double Fenv = 0.0;
double Fenv_old = 0.0;
int nProjectiles = 0;
boost::array<double,5> x_p = {{-4.0,-4.0,-4.0,-4.0,-4.0}};
double direction = 0.0;


// data structure and variables for logging experiment state
// TODO: once I figure out data logging with rosbags, I won't need this any more
const int MAX_SAMPLES = 200000;
int count = 0;
bool recordData = false;
double skin_stretch_position = 0.0;
double skin_stretch_setpoint = 0.0;
double skin_stretch_command = 0.0;
double tactor1_command = 0.0;
double tactor2_command = 0.0;
typedef struct {
  int subjectNumber;
  int trialNumber;
  int setNumber;
  int set_type;       // practice (0), experiment (1), or training (2)
  int visual;         // flag for visual feedback
  int vibrotactile;   // flag for vibrotactile feedback
  int skin_stretch;   // flag for skin stretch feedback
  double admittance;
  double deadband;
  double k1[MAX_SAMPLES];
  double k2[MAX_SAMPLES];
  double b1[MAX_SAMPLES];
  double b2[MAX_SAMPLES];
  double m1[MAX_SAMPLES];
  double m2[MAX_SAMPLES];
  double t[MAX_SAMPLES];
  double x_a[MAX_SAMPLES];
  double x_d1[MAX_SAMPLES];
  double x_d2[MAX_SAMPLES];
  double xd_a[MAX_SAMPLES];
  double xd_d1[MAX_SAMPLES];
  double xd_d2[MAX_SAMPLES];
  double Fu_raw[MAX_SAMPLES];
  double Fu[MAX_SAMPLES];
  double Fenv[MAX_SAMPLES];
  double skin_stretch_position[MAX_SAMPLES];
  double skin_stretch_setpoint[MAX_SAMPLES];
  double skin_stretch_command[MAX_SAMPLES];
  double tactor1_command[MAX_SAMPLES];
  double tactor2_command[MAX_SAMPLES];
} dataStruct;

dataStruct data;


void GainsCallback(const simulation_impedance_experiment::Gains::ConstPtr& msg) {
  imp1.k = msg->stiffness1;
  imp1.b = msg->damping1;
  imp1.m = msg->mass1;
  imp2.k = msg->stiffness2;
  imp2.b = msg->damping2;
  imp2.m = msg->mass2;
  //ROS_INFO("Received gains. k=%f, b=%f, m=%f",imp.k,imp.b,imp.m);
}

void Quit(const std_msgs::Bool::ConstPtr& msg) {
  ROS_INFO("Quit message received.");
  ros::shutdown();
}


void DAQOutputCallback(const simulation_impedance_experiment::DAQoutput::ConstPtr& msg) {
  // message contains voltage outputs on DAQ
  tactor1_command = msg->tactor1;
  tactor2_command = msg->tactor2;
  skin_stretch_command = msg->skin_stretch;
}


void UserInputCallback(const simulation_impedance_experiment::DAQinput::ConstPtr& msg) {
  // message from daq node contains user input force in Newtons.
  // calculate commanded velocity.
  Fu_raw = msg->force_raw;
  Fu = msg->force;
  if (msg->force > deadband) {
    xd_d1 = admittance * (msg->force - deadband); // admittance relationship between input force and desired velocity
  } else if (msg->force < -1*deadband) {
    xd_d1 = admittance * (msg->force + deadband); // admittance relationship between input force and desired velocity
  } else {
    xd_d1 = 0;
  }
  // also store some info for logging
  // TODO: when I figure out data logging with rosbags, this can go away.
  skin_stretch_position = msg->pot1;
  skin_stretch_setpoint = msg->pot1_setpoint;
  ROS_DEBUG("User input: %f N",msg->force);
}


// function for writing data to file.
// TODO: figure out data logging with rosbags, so I can get rid of this.

void WriteDataFile(int subjectNumber, int setNumber, int trialNumber) {

	FILE *output_file;
	char fname[60]; // file name plus path

	// Open file for writing
	sprintf(fname,"data/Subject%02d/Set%02d_Trial%02d.m",subjectNumber,setNumber,trialNumber);
	output_file = fopen(fname,"a");

  // print trial info
	fprintf(output_file, "subjectNumber = %d;\n",data.subjectNumber);
	fprintf(output_file, "setNumber = %d;\n",data.setNumber);
	fprintf(output_file, "set_type = %d; %% 0 for practice, 1 for experiment, 2 for training\n",data.set_type);
	fprintf(output_file, "trialNumber = %d;\n",data.trialNumber);
	fprintf(output_file, "admittance = %f;\n",data.admittance);
	fprintf(output_file, "deadband = %f;\n",data.deadband);

  //print feedback flags
	fprintf(output_file, "visual = %d;\n",data.visual);
	fprintf(output_file, "vibrotactile = %d;\n",data.vibrotactile);
	fprintf(output_file, "skin_stretch = %d;\n",data.skin_stretch);

  // print time stamps
	fprintf(output_file, "t = [");
	for (int k=0; k<count; k++) {
		fprintf(output_file,"%f, ",data.t[k]);
	}
	fprintf(output_file, "];\n\n");

  // print impedance levels
  fprintf(output_file, "imp_user.k = [");
	for (int j=0; j<count; j++) {
		fprintf(output_file,"%f, ",data.k1[j]);
	}
	fprintf(output_file, "];\n\n");
	fprintf(output_file, "imp_user.b = [");
	for (int j=0; j<count; j++) {
		fprintf(output_file,"%f, ",data.b1[j]);
	}
	fprintf(output_file, "];\n\n");
	fprintf(output_file, "imp_user.m = [");
	for (int j=0; j<count; j++) {
		fprintf(output_file,"%f, ",data.m1[j]);
	}
	fprintf(output_file, "];\n\n");
  fprintf(output_file, "imp_env.k = [");
	for (int j=0; j<count; j++) {
		fprintf(output_file,"%f, ",data.k2[j]);
	}
	fprintf(output_file, "];\n\n");
	fprintf(output_file, "imp_env.b = [");
	for (int j=0; j<count; j++) {
		fprintf(output_file,"%f, ",data.b2[j]);
	}
	fprintf(output_file, "];\n\n");
	fprintf(output_file, "imp_env.m = [");
	for (int j=0; j<count; j++) {
		fprintf(output_file,"%f, ",data.m2[j]);
	}
	fprintf(output_file, "];\n\n");


  // print user input force (raw and filtered)

	fprintf(output_file, "Fu_raw = [");
  for (int j=0; j<count; j++) {
    fprintf(output_file,"%f, ",data.Fu_raw[j]);
  }
  fprintf(output_file, "];\n\n");

	fprintf(output_file, "Fu = [");
  for (int j=0; j<count; j++) {
    fprintf(output_file,"%f, ",data.Fu[j]);
  }
  fprintf(output_file, "];\n\n");


  // print state and feedback information
	// desired user position
	fprintf(output_file, "x_d1 = [");
  for (int j=0; j<count; j++) {
    fprintf(output_file,"%f, ",data.x_d1[j]);
  }
  fprintf(output_file, "];\n\n");

	// desired user velocity
	fprintf(output_file, "xd_d1 = [");
  for (int j=0; j<count; j++) {
    fprintf(output_file,"%f, ",data.xd_d1[j]);
  }
  fprintf(output_file, "];\n\n");

  // desired moving object position
	fprintf(output_file, "x_d2 = [");
  for (int j=0; j<count; j++) {
    fprintf(output_file,"%f, ",data.x_d2[j]);
  }
  fprintf(output_file, "];\n\n");

	// desired moving object velocity
	fprintf(output_file, "xd_d2 = [");
  for (int j=0; j<count; j++) {
    fprintf(output_file,"%f, ",data.xd_d2[j]);
  }
  fprintf(output_file, "];\n\n");

	// actual position
	fprintf(output_file, "x_a = [");
  for (int j=0; j<count; j++) {
    fprintf(output_file,"%f, ",data.x_a[j]);
  }
  fprintf(output_file, "];\n\n");

  // actual velocity
  fprintf(output_file, "xd_a = [");
  for (int j=0; j<count; j++) {
    fprintf(output_file,"%f, ",data.xd_a[j]);
  }
  fprintf(output_file, "];\n\n");

  // environment force
	fprintf(output_file, "Fenv = [");
  for (int j=0; j<count; j++) {
    fprintf(output_file,"%f, ",data.Fenv[j]);
  }
  fprintf(output_file, "];\n\n");

  if (data.skin_stretch) {

    // skin stretch position, for evaluating motor performance
    fprintf(output_file, "skin_stretch_position = [");
    for (int j=0; j<count; j++) {
      fprintf(output_file,"%f, ",data.skin_stretch_position[j]);
    }
    fprintf(output_file, "];\n\n");

    // skin stretch setpoint, for evaluating motor performance
    fprintf(output_file, "skin_stretch_setpoint = [");
    for (int j=0; j<count; j++) {
      fprintf(output_file,"%f, ",data.skin_stretch_setpoint[j]);
    }
    fprintf(output_file, "];\n\n");

    // skin stretch command output, for evaluating motor performance
    fprintf(output_file, "skin_stretch_command = [");
    for (int j=0; j<count; j++) {
      fprintf(output_file,"%f, ",data.skin_stretch_command[j]);
    }
    fprintf(output_file, "];\n\n");
  }

  if (data.vibrotactile) {

    // tactor 1 command
    fprintf(output_file, "tactor1_command = [");
    for (int j=0; j<count; j++) {
      fprintf(output_file,"%f, ",data.tactor1_command[j]);
    }
    fprintf(output_file, "];\n\n");
    
    // tactor 2 command
    fprintf(output_file, "tactor2_command = [");
    for (int j=0; j<count; j++) {
      fprintf(output_file,"%f, ",data.tactor2_command[j]);
    }
    fprintf(output_file, "];\n\n");

  }

	fclose(output_file);

}


void ClearArrays(void) {
  ROS_INFO("Clearing data arrays...");
	memset(data.t,0,MAX_SAMPLES*sizeof(double));
	memset(data.Fenv,0,MAX_SAMPLES*sizeof(double));
	memset(data.x_d1,0,MAX_SAMPLES*sizeof(double));
	memset(data.xd_d1,0,MAX_SAMPLES*sizeof(double));
	memset(data.x_a,0,MAX_SAMPLES*sizeof(double));
	memset(data.xd_a,0,MAX_SAMPLES*sizeof(double));
	memset(data.x_d2,0,MAX_SAMPLES*sizeof(double));
	memset(data.xd_d2,0,MAX_SAMPLES*sizeof(double));
	memset(data.k1,0,MAX_SAMPLES*sizeof(double));
	memset(data.b1,0,MAX_SAMPLES*sizeof(double));
	memset(data.m1,0,MAX_SAMPLES*sizeof(double));
	memset(data.k2,0,MAX_SAMPLES*sizeof(double));
	memset(data.b2,0,MAX_SAMPLES*sizeof(double));
	memset(data.m2,0,MAX_SAMPLES*sizeof(double));
	memset(data.Fu,0,MAX_SAMPLES*sizeof(double));
	memset(data.Fu_raw,0,MAX_SAMPLES*sizeof(double));
	memset(data.skin_stretch_position,0,MAX_SAMPLES*sizeof(double));
	memset(data.skin_stretch_setpoint,0,MAX_SAMPLES*sizeof(double));
	memset(data.skin_stretch_command,0,MAX_SAMPLES*sizeof(double));
	memset(data.tactor1_command,0,MAX_SAMPLES*sizeof(double));
	memset(data.tactor2_command,0,MAX_SAMPLES*sizeof(double));
  ROS_INFO("Done.");
}


// initially, the simulation is not running
bool move = false;

bool startSimulation(simulation_impedance_experiment::StartSimulation::Request  &req,
                     simulation_impedance_experiment::StartSimulation::Response &res )
{
  move = true;
  x_a = 0.0;
  x_d1 = 0.0;
  x_d2 = 0.0;
  xd_a = 0.0;
  xd_d1 = 0.0;
  xd_d2 = 0.0;
  Fenv = 0.0;
  x_a_old = 0.0;
  x_d1_old = 0.0;
  x_d2_old = 0.0;
  xd_a_old = 0.0;
  xd_d1_old = 0.0;
  xd_d2_old = 0.0;
  Fenv_old = 0.0;
  for (int i=0;i<5;i++) {
    x_p[i] = -4.0;
  }
  task = (taskType)req.task;
  set_type = (setType)req.set_type;
  direction = req.direction;
  *t_start = ros::Time::now();

  // store data for logging
  data.trialNumber = req.trialNumber;
  data.subjectNumber = req.subjectNumber;
  data.setNumber = req.setNumber;
  data.set_type = req.set_type;
  data.visual = (int)req.visual;
  data.vibrotactile = (int)req.vibrotactile;
  data.skin_stretch = (int)req.skin_stretch;

  ClearArrays();
  count = 0;
  recordData = true;

  ROS_INFO("Simulation started.");
  return true;
}

bool resetSimulation(std_srvs::Empty::Request  &req,
                     std_srvs::Empty::Response &res )
{
  move = false;
  recordData = false;
  Fenv = 0.0;

  ROS_INFO("Writing data file for Subject %02d, Set %02d, Trial %02d...",data.subjectNumber,data.setNumber,data.trialNumber);
  WriteDataFile(data.subjectNumber,data.setNumber,data.trialNumber);
  ROS_INFO("Done.");

  ROS_INFO("Simulation reset.");
  return true;
}



int main(int argc, char **argv) {

  // This node simulates the 1-dof virtual prosthesis. It's really just a diff eq solver.


  // constants to define sinusoid for x_d1
  const double amp1 = 0.8;
  const double amp2 = -0.6;
  const double freq1 = 1.2;
  const double freq2 = 1.9;

  // constant to define position for feedback trainng
  const double position = 1.0;

  // define admittance and deadband for logging
  data.admittance = admittance;
  data.deadband = deadband;

  // initialize gains
  imp1.m = 1.5; // mass of virtual prosthesis
  imp2.m = 1.5;// mass of moving object in LI task
  imp1.k = 20; // stiffness of virtual prosthesis
  imp2.k = 20; // stiffness of moving object in LI task
  imp1.b = 2.5; // viscosity of virtual prosthesis
  imp2.b = 2.5; // viscosity of moving object in LI task

  //double mp = 1; // mass of projectile in HI task

  // set up ROS stuff
  ros::init(argc, argv, "simulation");
  ros::NodeHandle n;
  ros::Rate loop_rate(1000); // publishing rate in Hz
  ros::Time t;
  ros::Time told;
  ros::Duration deltat;
  t_start = new ros::Time;
  *t_start = ros::Time::now();

  // subscriber for receiving messages about user and moving object gains
  ros::Subscriber sub_gains = n.subscribe<simulation_impedance_experiment::Gains>("gains", 1000, GainsCallback);
  // subscriber for getting user command via daq
  ros::Subscriber sub_user_input = n.subscribe<simulation_impedance_experiment::DAQinput>("daq_input", 1000, UserInputCallback);
  // subscriber for getting daq output for data logging
  ros::Subscriber sub_daq_output = n.subscribe<simulation_impedance_experiment::DAQoutput>("daq_output", 1000, DAQOutputCallback);
  // subscriber for receiving quit command
  ros::Subscriber sub_quit = n.subscribe<std_msgs::Bool>("quit", 1000, Quit);

  // publisher for sending messages about state info
  ros::Publisher experiment_state_pub = n.advertise<simulation_impedance_experiment::ExperimentState>("experiment_state", 1000);
  simulation_impedance_experiment::ExperimentState msg_experiment_state;

  // services for starting and resetting the simulation
  ros::ServiceServer server_start_simulation = n.advertiseService("start_simulation", startSimulation);
  ros::ServiceServer server_reset_simulation = n.advertiseService("reset_simulation", resetSimulation);


  // task type
  taskType task = forcemin;

  // Declare vectors and matrices for solving diff eq
  gsl_vector * X = gsl_vector_alloc(2);
  gsl_matrix * A = gsl_matrix_alloc(2,2);
  gsl_matrix * Adt = gsl_matrix_alloc(2,2);
  //gsl_matrix * I = gsl_matrix_calloc(2,2);
  gsl_vector * C = gsl_vector_alloc(2);
  gsl_vector * C2 = gsl_vector_alloc(2);
  gsl_vector * H = gsl_vector_alloc(2);
  gsl_matrix * eA = gsl_matrix_alloc(2,2);
  gsl_permutation * p = gsl_permutation_alloc (2);
  int status = 0; // this can be used to get error messages from matrix operations (TODO: set up error checking)
  int s;


  // Loop:
  // Calculate actual motion of moving object/virtual prosthesis, and publish message.

  while (ros::ok()) {

  ROS_DEBUG("Gains 1. k=%f, b=%f, m=%f",imp1.k,imp1.b,imp1.m);
  ROS_DEBUG("Gains 2. k=%f, b=%f, m=%f",imp2.k,imp2.b,imp2.m);
    // get timestamp and period
    told = t;
    t = ros::Time::now();
    deltat = t - told;
    msg_experiment_state.t = t;
    msg_experiment_state.deltat = deltat;

    // Solve the differential equation:
    //	(m1 + m2) xdd_a = -b1(xd_a - xd_d1) - k1(x_a - x_d1) - b2(xd_a - xd_d2) - k2(x_a - x_d2)
    // for LI task, or
    //	m1 xdd_a = -k1(x_a - x_d1) - b1(xd_a - xd_d1) + Fenv
    // for HI task
    // where everything is known except x_a, xd_a, xdd_a.
    // Rewrite in the form:
    //	M xdd_a + B xd_a + K x_a = F
    double M;
    double B;
    double K;
    double F;
    double Fprev;

    if (move) {

      // calculate desired position of user.
      // desired velocity is specified by force input when message is received from daq node
			x_d1 = x_d1 + (xd_d1 + xd_d1)/2*deltat.toSec(); // approx integral of xd_des with sum
      // calculate F in the diff eq
      if (task == forcemin) {
        
        if (set_type == training) {
          // constant position for feedback training
          x_d2 = direction*position;
          xd_d2 = 0.0;
        } else {
          // sinusoid otherwise
          x_d2 = direction*(amp1*sin(freq1*(t - *t_start).toSec()) + amp2*sin(freq2*(t - *t_start).toSec()));
          xd_d2 = direction*(amp1*freq1*cos(freq1*(t - *t_start).toSec()) + amp2*freq2*cos(freq2*(t - *t_start).toSec()));
        }

				M = imp1.m + imp2.m;
				B = imp1.b + imp2.b;
				K = imp1.k + imp2.k;
				F = imp1.b*xd_d1 + imp2.b*xd_d2 + imp1.k*x_d1 + imp2.k*x_d2;
				Fprev = imp1.b*xd_d1_old + imp2.b*xd_d2_old + imp1.k*x_d1_old + imp2.k*x_d2_old;

      } else if (task == tracking) {
        
        M = imp1.m;
        B = imp1.b;
        K = imp1.k;
				// Deal with possible perturbation. We model the perturbation as an impact with a projectile, and we display the projectile's motion to give the user some warning.
        Fenv = 0;
        //TODO: collisions
				F = Fenv + imp1.k * x_d1 + imp1.b * xd_d1;
				Fprev = Fenv_old + imp1.k * x_d1_old + imp1.b * xd_d1_old;

      } else {

        //TODO: what if undefined task?

      }

      // Now rewrite in matrix form:
			// 	Xd = AX + Cu
			// where X is the vector [x_a; xd_a]
			// Define X at the previous timestep
			gsl_vector_set (X, 0, x_a_old);
			gsl_vector_set (X, 1, xd_a_old);
			// Define A, C
			gsl_matrix_set (A, 0, 0, 0.0);
			gsl_matrix_set (A, 0, 1, 1.0);
			gsl_matrix_set (A, 1, 0, -K/M);
			gsl_matrix_set (A, 1, 1, -B/M);
			gsl_vector_set (C, 0, 0);
			gsl_vector_set (C, 1, 1);
			status = gsl_vector_memcpy(C2,C);
			gsl_matrix_set (Adt, 0, 0, 0.0);
			gsl_matrix_set (Adt, 0, 1, deltat.toSec());
			gsl_matrix_set (Adt, 1, 0, -K*deltat.toSec()/M);
			gsl_matrix_set (Adt, 1, 1, -B*deltat.toSec()/M);

			// Solve the diff eq for the current timestep.
			// With the assumption that u is piecewise constant, i.e., constant over the interval [t, t+dt), the solution is
			//	X(t + dt) = exp(A*dt) * X(t) + H * u(t)
			// where H = inv(A) * [exp(A*dt) - I] * C.
			// Here, since we know u is not really constant, we will take the value of u to be the average of the readings at the current and previous timesteps. So u = 0.5*(u[count-1] + u[count]).
			double u = 0.5*(F+Fprev)/M;
			// Calculate the matrix exponential
			status = gsl_linalg_exponential_ss(Adt, eA, GSL_PREC_DOUBLE);
			// Calculate [exp(Adt) - I] * C. The answer is stored in C, which we don't need any more anyway.
			status = gsl_blas_dgemv(CblasNoTrans, 1, eA, C2, -1, C);
			// Get an LU decomposition of A (needed to take the inverse). The answer will be stored in A, which is ok because we don't need A any more (we just need eA). See page 128 of gsl-ref.pdf.
			status = gsl_linalg_LU_decomp(A, p, &s);
			// Finally calculate H!
			status = gsl_linalg_LU_solve(A, p, C, H);
			// Now calcuate X at the current timestep!! The answer is stored in H, unfortunately that's the way the function works.
			status = gsl_blas_dgemv(CblasNoTrans, 1, eA, X, u, H);
			// Pull out x_a and xd_a
			x_a = gsl_vector_get(H,0);
			xd_a = gsl_vector_get(H,1);

			if (task == forcemin) {
				// Calculate environment force
				Fenv = -1 * imp2.b*(xd_a - xd_d2) - imp2.k*(x_a - x_d2);
      }

      x_a_old = x_a;
      x_d1_old = x_d1;
      x_d2_old = x_d2;
      xd_a_old = xd_a;
      xd_d1_old = xd_d1;
      xd_d2_old = xd_d2;
      Fenv_old = Fenv;

    }
 
    // put data into experiment state message
    msg_experiment_state.task = (int)task;
    msg_experiment_state.x_a = x_a;
    msg_experiment_state.x_d1 = x_d1;
    msg_experiment_state.x_d2 = x_d2;
    msg_experiment_state.xd_a = xd_a;
    msg_experiment_state.xd_d1 = xd_d1;
    msg_experiment_state.xd_d2 = xd_d2;
    msg_experiment_state.Fenv = Fenv;
    msg_experiment_state.direction = direction;
    msg_experiment_state.nProjectiles = nProjectiles;
    msg_experiment_state.x_p = x_p;

    // publish experiment state message
    experiment_state_pub.publish(msg_experiment_state);


    // save data for logging
    if (recordData) {
      data.k1[count] = imp1.k;
      data.k2[count] = imp2.k;
      data.b1[count] = imp1.b;
      data.b2[count] = imp2.b;
      data.m1[count] = imp1.m;
      data.m2[count] = imp2.m;
      data.t[count] = t.toSec();
      data.Fu[count] = Fu;
      data.Fu_raw[count] = Fu_raw;
      data.x_a[count] = x_a;
      data.x_d1[count] = x_d1;
      data.x_d2[count] = x_d2;
      data.xd_a[count] = xd_a;
      data.xd_d1[count] = xd_d1;
      data.xd_d2[count] = xd_d2;
      data.Fenv[count] = Fenv;
      data.skin_stretch_position[count] = skin_stretch_position;
      data.skin_stretch_setpoint[count] = skin_stretch_setpoint;
      data.tactor1_command[count] = tactor1_command;
      data.tactor2_command[count] = tactor2_command;
      data.skin_stretch_command[count] = skin_stretch_command;
      count++;
      if (count > MAX_SAMPLES) {
        ROS_ERROR("Data logging exceeded array size. Overwriting data array.");
        count = 0;
      }
    }

    ros::spinOnce();
    loop_rate.sleep();

  }

 	gsl_matrix_free(A);
	gsl_matrix_free(eA);
	gsl_matrix_free(Adt);
	//gsl_matrix_free(I);
	gsl_vector_free(X);
	gsl_vector_free(H);
	gsl_vector_free(C);
	gsl_vector_free(C2);
	gsl_permutation_free(p);
	 
  return 0;
}
