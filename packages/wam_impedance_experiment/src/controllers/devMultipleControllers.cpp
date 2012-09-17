#include "ros/ros.h"
#include <iostream>
#include <cisstOSAbstraction/osaGetTime.h>
#include <cisstMultiTask/mtsInterfaceProvided.h>
#include <cisstMultiTask/mtsInterfaceRequired.h>
#include "wam_impedance_experiment/controllers/devMultipleControllers.h"

// gsl stuff for finding the nullspace of the Jacobian
#include <gsl/gsl_vector.h>
#include <gsl/gsl_matrix.h>
#include <gsl/gsl_blas.h>
#include <gsl/gsl_errno.h>
#include <gsl/gsl_linalg.h>

#include "ros/ros.h"


CMN_IMPLEMENT_SERVICES(ControllerData);
CMN_IMPLEMENT_SERVICES(ControlInfo);

devMultipleControllers::devMultipleControllers(const std::string& name, 
					       double period,
					       devController::State state,
					       osaCPUMask mask,
					       const std::string& robfile,
					       const vctFrame4x4<double>& Rtw0,
                 const vctDynamicMatrix<double>& Kp_Rn,
                 const vctDynamicMatrix<double>& Kd_Rn,
                 vctFixedSizeMatrix<double,6,6>& Kp_SE3,
                 vctFixedSizeMatrix<double,6,6>& Kd_SE3 ):

  devController( name, period, state, mask ),
  robManipulator_AAB( robfile, Rtw0 ),
  name(name),
  inputSE3( NULL ),
  inputRn( NULL ),
  output( NULL ),
  feedback( NULL ),
  controller( gravityCompensation ),
  Kp_Rn( Kp_Rn ),
  Kd_Rn( Kd_Rn ),
  Kp_SE3( Kp_SE3 ),
  Kd_SE3( Kd_SE3 ),
  Kp_null(0.0),
  Kd_null(0.0),
  friction_gain(0.0),
  impulse(0.0),
  impulseTime(0.07),
  nImpulseSteps(10),
  countImpulseSteps(0),
  dt_nominal(0),
  theta(0.0),
  told(-1.0),
  told_fb ( -1.0 ),
  transitionTime(3.0),
  t_transition(-1.0),
  newController ( false ),
  tracking_goal(0.0) {
  
  dt_nominal = GetPeriodicity();
  nImpulseSteps = ceil(impulseTime/dt_nominal);
  impulseTime = nImpulseSteps*dt_nominal;
  forceExternal.SetSize( nImpulseSteps );
  forceExternal.SetAll( 0.0 );
  qold.SetSize( links.size() );
  qold.SetAll( 0.0 );
  setpoint_null.SetSize( links.size() );
  setpoint_null.SetAll( 0.0 );
  //setpoint_null[2] = 0.8;
  //setpoint_null[2] = -0.9713;
  qdold.SetSize( links.size() );
  qdold.SetAll( 0.0 );
  qdfiltold.SetSize( links.size() );
  qdfiltold.SetAll( 0.0 );
  eold.SetSize( links.size() );
  eold.SetAll( 0.0 );
  erroraxis.SetAll( 0.0 );
  tauold.SetSize( links.size() );
  tauold.SetAll( 0.0 );
  tau_gc.SetSize( links.size() );
  tau_gc.SetAll( 0.0 );
  tau_ct.SetSize( links.size() );
  tau_ct.SetAll( 0.0 );
  tau_imp.SetSize( links.size() );
  tau_imp.SetAll( 0.0 );
  tau_imp_null.SetSize( links.size() );
  tau_imp_null.SetAll( 0.0 );
  tau_imp_null_fric.SetSize( links.size() );
  tau_imp_null_fric.SetAll( 0.0 );
  tau_external.SetSize( links.size() );
  tau_external.SetAll( 0.0 );
  tau_transition.SetSize( links.size() );
  tau_transition.SetAll( 0.0 );
  saturation_limits[0] = 65;
  saturation_limits[1] = 60;
  saturation_limits[2] = 25;
  saturation_limits[3] = 25;
  saturation_limits[4] = 10;
  saturation_limits[5] = 10;
  saturation_limits[6] = 5;
  
  // input from trajectory for jointspace positions
  inputRn = RequireInputRn( "InputRn",
            devRobotComponent::POSITION |
            devRobotComponent::VELOCITY,
            links.size() );

  // input from trajectory for Cartesian positions and orientations
  inputSE3 = RequireInputSE3( "InputSE3",
             devRobotComponent::POSITION | 
             devRobotComponent::VELOCITY );

  // output joint torques to wam
  output = RequireOutputRn( devController::Output,
			    devRobotComponent::FORCETORQUE,
			    links.size() );

  // joint position feedback from wam
  feedback = RequireInputRn( devController::Feedback,
			     devRobotComponent::POSITION,
			     links.size() );


  // add interface for switching control laws
  mtsInterfaceProvided* control_selection_interface = AddInterfaceProvided( "ControlInfo" );
  if (control_selection_interface) {
    control_selection_interface->AddCommandWrite( &devMultipleControllers::SetControllerInfo,
                                                  this,
                                                  "ControlInfoCommand" );
  }

  // add interface for changing controller gains
  mtsInterfaceProvided* gains_se3_interface = AddInterfaceProvided( "Gains" );
  if (gains_se3_interface) {
    gains_se3_interface->AddCommandWrite( &devMultipleControllers::SetGains,
                                          this,
                                          "GainsCommand" );
  }

  // add interface for adding an external impulse to simulate impact
  mtsInterfaceProvided* impulse_interface = AddInterfaceProvided( "Impulse" );
  if (impulse_interface) {
    impulse_interface->AddCommandWrite( &devMultipleControllers::AddImpulse,
                                          this,
                                          "ImpulseCommand" );
  }

  // add interface getting tracking goal from graphics node
  mtsInterfaceProvided* tracking_goal_interface = AddInterfaceProvided( "TrackingGoal" );
  if (tracking_goal_interface) {
    tracking_goal_interface->AddCommandWrite( &devMultipleControllers::GetTrackingGoal,
                                          this,
                                          "TrackingGoalCommand" );
  }

  // add interface for publishing all sorts of data. this is a required interface to force publishing every loop.
  mtsInterfaceRequired* data_publisher_interface = AddInterfaceRequired( "ControllerDataPublisher" );
  if (data_publisher_interface) {
    data_publisher_interface->AddFunction( "ControllerDataPublishFunction",
                                            PublishControllerData );
  }

  // interface to tell trajectory to reset when controller type is switched.
  /*
  mtsInterfaceRequired* trajectory_reset_interface = AddInterfaceRequired("TrajectoryResetInterface");
  if (trajectory_reset_interface) {
    trajectory_reset_interface->AddFunction( "TrajectoryReset",
                                             ResetTrajectory );
  }
  */

  // interface so the trajectory can tell the controller when it is finished dealing with controller switch
  /*
  mtsInterfaceProvided* trajectory_reset_done_interface = AddInterfaceProvided("TrajectoryResetDoneInterface");
  if (trajectory_reset_done_interface) {
    trajectory_reset_done_interface->AddCommandVoid( &devMultipleControllers::ResetTrajectoryDone,
                                                     this,
                                                     "TrajectoryResetDone");
  }
  */

}


/****************************************************************************************
 *
 * Function for changing controller types, activated by receiving a message
 * containing control info.
 *
 * data.controllerNumber specifies which control law to use
 * data.stiffness and data.damping are the variable impedance parameters
 *
 ****************************************************************************************/

void devMultipleControllers::SetControllerInfo (const ControlInfo& data) {

  /*
  if (controller != (controllerType)(data.controllerNumber)) {
    // switching controller types so set a flag to note that this timestep is special,
    // and then tell the trajectory to reset.
    newController = true;
    eold.SetAll( 0.0 );
    //ResetTrajectory();
  }
  */

  if (controller != (controllerType)(data.controllerNumber)) {
    controller = (controllerType)(data.controllerNumber);
    newController = true;
    t_transition = osaGetTime();
    tau_transition = tauold;

    // calculate appropriate transition time based on torque change
    //double scale = 20.0; // N-m per second
    vctDynamicVector<double> dtau(7,0.0);
    if (controller == gravityCompensation) {
      dtau = tau_gc - tau_transition;
    } else if (controller == computedTorque_Rn) {
      dtau = tau_ct - tau_transition;
    } else if (controller ==  impedance_SE3) {
      dtau = tau_imp - tau_transition;
    } else if (controller ==  impedance_SE3_null) {
      dtau = tau_imp_null - tau_transition;
    } else if (controller ==  impedance_SE3_null_fric) {
      dtau = tau_imp_null_fric - tau_transition;
    }
    double dtau_max = 0.0;
    for (unsigned int i=0;i<links.size();i++) {
      // grab largest entry of dtau
      if (fabs(dtau[i]) > dtau_max) {
        dtau_max = fabs(dtau[i]);
      }
    }
    //transitionTime = dtau_max/scale;
    ROS_INFO("Beginning controller transition: %f seconds.", transitionTime);
  } else {
    controller = (controllerType)(data.controllerNumber);
  }
  if ((controller == impedance_SE3_null) || (controller == impedance_SE3_null_fric)) {
    setpoint_null = data.setpoint_null;
  }

}

/****************************************************************************************
 *
 * Function for changing controller gains, activated by receiving a message
 * containing gains.
 *
 ****************************************************************************************/

void devMultipleControllers::SetGains(const mtsVector<double>& data) {

  for (int i=0;i<7;i++) {
    Kp_Rn[i][i] = data[i];
    Kd_Rn[i][i] = data[i+7];
    if (i < 6) {
      Kp_SE3[i][i] = data[i+14];
      Kd_SE3[i][i] = data[i+20];
    }
  }
  Kp_null = data[26];
  Kd_null = data[27];
  friction_gain = data[28];

}

/****************************************************************************************
 *
 * Function for adding an externally specified impulse to simulate impact.
 * 
 ****************************************************************************************/

void devMultipleControllers::AddImpulse(const double& data) {
  impulse = data;
  double force = impulse/impulseTime;
  for (unsigned int i=0;i<forceExternal.size();i++) {
    forceExternal[i] += force;
  }
}

/****************************************************************************************
 *
 * Function for receiving tracking goal from graphics node
 * 
 ****************************************************************************************/

void devMultipleControllers::GetTrackingGoal(const double& data) {
  tracking_goal = data;
}

/****************************************************************************************
 *
 * Runs every timestep when the controller is enabled.
 * 
 ****************************************************************************************/

void devMultipleControllers::Evaluate() {

  // need two sets of timestamps: one for input coming from the wam feedback
  // and the other for input coming from the trajectory.
  double t;
  double t_fb;
  double t_traj;
  double dt;
  double dt_fb = 0;

  // current time
  t = osaGetTime();
  dt = t - told;

  // robot state
  vctDynamicVector<double> q(links.size(), 0.0);
  vctDynamicVector<double> qd(links.size(), 0.0);   // zero velocity
  vctDynamicVector<double> qdd(links.size(), 0.0);  // zero acceleration
  vctFrame4x4<double> Rtwn;

  // commands from trajectory
  vctDynamicVector<double> qs(links.size(), 0.0);
  vctDynamicVector<double> qsd(links.size(), 0.0);
  vctDynamicVector<double> qsdd(links.size(), 0.0);
  vctFrame4x4<double> Rtwns;
  vctFixedSizeVector<double,6> xds(0.0);    // 6-vector of desired translational and rotational end effector velocities -
                                            // note that this is NOT the vw vector from MLS, this is just world velocities
                                            // of the end effector.

  // commanded torque output
  vctDynamicVector<double> tau(links.size(), 0.0);

  // get current robot state

  // Fetch the position
  feedback->GetPosition( q, t_fb );
  if( q.size() != links.size() ){
    CMN_LOG_RUN_ERROR << CMN_LOG_DETAILS
		      << "size(q) = " << q.size() << " "
		      << "N = " << links.size() << std::endl;
  }
  Rtwn = ForwardKinematics(q); // returns homogeneous transformation from end effector frame to world frame for joint angles q

  dt_fb = t_fb - told_fb;

  // Evaluate the velocity
  if (told_fb > 0) {
    // only calculate a new velocity if there is new data read from the state table.
    // otherwise, use the velocity from the previous loop.
    if (dt_fb > 0) {
      qd = (q - qold)/dt_fb;
    } else {
      qd = qdold;
    }
  }

  // Inertia matrix at q
  vctDynamicMatrix<double> M_JS = JSinertia( q );
  // operational space inertia
  double M_OS[6][6];
  // end effector inertia in world frame
  vctFixedSizeMatrix<double,3,3> M(0.0);

  // Compute the coriolis+gravity load
  vctDynamicVector<double> ccg = CCG( q, qd );

  inputRn->GetPosition( qs, t_traj );
  if( qs.size() != links.size() ){
    CMN_LOG_RUN_ERROR << CMN_LOG_DETAILS
      << "size(q) = " << qs.size() << " "
      << "N = " << links.size() << std::endl;
  }
  inputSE3->GetPosition( Rtwns, t_traj );
  inputSE3->GetVelocity( xds, t_traj );

  if(q.size() == links.size()) {

    // calculate torques for all controllers

    // -----------------------
    // gravity compensation

    tau_gc =  InverseDynamics( q, qd, qdd );

    // -----------------------
    // computed torque Rn

    if (qs.size() == links.size()) {

      // evaluate the velocity
      // error = actual - desired
      vctDynamicVector<double> e = q - qs;

      // error time derivative
      vctDynamicVector<double> ed( links.size(), 0.0 );
      if( 0 < dt ) {
        ed = (e - eold)/dt;      
      }

      // compute the augmented PD output
      tau_ct = M_JS*( qsdd - Kp_Rn*e - Kd_Rn*ed ) + ccg;

      eold = e;

    } else {

      tau_ct = tau_gc;

    }

    // -----------------------
    // impedance SE3

    vctFixedSizeVector<double,6> xd(0.0);
    vctFixedSizeVector<double,6> xd_unfilt(0.0);
    double cutoff = 1000;
    vctFixedSizeVector<double,6> vw(0.0);     // 6-vector of translational and rotational end effector velocities - 
    // note that this is the vw vector used in MLS, see pp 54-55 for interpretation.
    vctFixedSizeVector<double,6> FM(0.0);
    vctFixedSizeVector<double,3> xs = Rtwns.Translation();
    vctFixedSizeVector<double,3> x(0.0);
    // Get actual position of robot in Cartesian space, world frame
    x = Rtwn.Translation();

    JacobianBody(q); // updates the body Jacobian
    JacobianSpatial(q); // updates the spatial Jacobian
    // get inertia at the end effector in the end effector frame.
    OSinertia(M_OS,q);
    // get inertia at the end effector in the world frame.
    vctFixedSizeMatrix<double,3,3> R(Rtwn.Rotation());
    for (int i=0;i<3;i++) {
      for (int j=0;j<3;j++) {
        M[i][j] = M_OS[i][j];
      }
    }
    M = R*M*R.Transpose();

    // vw = Js*qd;
    // need to do the multiplication by hand.
    // also note that Js is in column major (switch order of row and column indices)
    for (int j=0;j<6;j++) {
      vw[j] = 0.0;
      for (unsigned int i=0;i<links.size();i++) {
        vw[j] += Js[i][j]*qd[i];
      }
    }

    xd[0] = vw[4]*x[2] - vw[5]*x[1] + vw[0];
    xd[1] = vw[5]*x[0] - vw[3]*x[2] + vw[1];
    xd[2] = vw[3]*x[1] - vw[4]*x[0] + vw[2];
    xd[3] = vw[3];
    xd[4] = vw[4];
    xd[5] = vw[5];

    // filter velocity - y-axis angular velocity only
    xd_unfilt = xd;
    cutoff = 5;
    double beta = exp(-1.0*cutoff*dt);
    xd[4] = beta*xdold[4] + (1.0 - beta)*xd_unfilt[4];

    // error = current - desired
    vctFixedSizeVector<double,3> ex = x - xs;			     // 3-vector for just position error
    vctFixedSizeVector<double,3> ea( 0.0 );			     // 3-vector for just orientation error
    ea[0] = -0.5*( Rtwn[1][0]*Rtwns[2][0] - Rtwn[2][0]*Rtwns[1][0] + Rtwn[1][1]*Rtwns[2][1] - Rtwn[2][1]*Rtwns[1][1] + Rtwn[1][2]*Rtwns[2][2] - Rtwn[2][2]*Rtwns[1][2] );
    ea[1] = -0.5*( Rtwn[2][0]*Rtwns[0][0] - Rtwn[0][0]*Rtwns[2][0] + Rtwn[2][1]*Rtwns[0][1] - Rtwn[0][1]*Rtwns[2][1] + Rtwn[2][2]*Rtwns[0][2] - Rtwn[0][2]*Rtwns[2][2] );
    ea[2] = -0.5*( Rtwn[0][0]*Rtwns[1][0] - Rtwn[1][0]*Rtwns[0][0] + Rtwn[0][1]*Rtwns[1][1] - Rtwn[1][1]*Rtwns[0][1] + Rtwn[0][2]*Rtwns[1][2] - Rtwn[1][2]*Rtwns[0][2] );
    vctFixedSizeVector<double,6> e( ex[0],ex[1],ex[2],ea[0],ea[1],ea[2] ); // 6-vector for position and orientation error 
    vctFixedSizeVector<double,6> ed = xd - xds;

    // Calculate control force in Cartesian space, world frame
    // option a: use the approximation, x*sin(theta)
    //FM = -1.0*Kp_SE3*e - Kd_SE3*ed;

    // option b: use x*theta
    vctFrame4x4<double> RtwError = Rtwns*Rtwn.Transpose();
    // cos(theta) = 0.5*(tr(R)-1)
    double costheta = 0.5*(RtwError[0][0] + RtwError[1][1] + RtwError[2][2] - 1);
    // abs(sin(theta)) = sqrt(ea'*ea)
    double sinthetaabs = pow((ea[0]*ea[0] + ea[1]*ea[1] + ea[2]*ea[2]),0.5);
    // assume sin(theta) positive
    theta = atan2(sinthetaabs,costheta);
    // find x (axis of rotation for orientation error)
    erroraxis = ea/sinthetaabs;
    // find error x*theta
    vctFixedSizeVector<double,3> eb( 0.0 );
    eb = erroraxis*theta;
    for (int i=0;i<3;i++) {
      e[i+3] = eb[i];
    }
    FM = -1.0*Kp_SE3*e - Kd_SE3*ed;

    // Note that the forces and moments are applied at the end effector but represented in the world frame.
    // Need to convert to a wrench in spatial frame. We can consider FM to be a wrench in a frame that is
    // oriented with the spatial frame but located at the end effector.
    //
    // Convert to equivalent wrench in spatial frame - the frames are just translated
    // no change to forces, but M = M + x cross F
    FM[3] = x[1]*FM[2] - x[2]*FM[1] + FM[3];
    FM[4] = x[2]*FM[0] - x[0]*FM[2] + FM[4];
    FM[5] = x[0]*FM[1] - x[1]*FM[0] + FM[5];

    // Convert FM to joint torques
    // tau = J'*[F;M]
    // Since FM is in the world frame, use the spatial Jacobian
    for (unsigned int i=0;i<links.size();i++) {
      tau_imp[i] = 0.0;
      for (int j=0;j<6;j++) {
        tau_imp[i] += Js[i][j]*FM[j];
      }
    }

    tau_imp += ccg;

    // add nullspace PD
    //
    // first, compute the null space of the Jacobian. since there is only one redundant
    // dof, there is a closed form solution. (this should calculate the nullspace associated
    // with the redundant dof; if the arm is in a singular configuration, there will be
    // one or more other vectors associated with the nullspace.)

    vctDynamicVector<double> nullspace(7,0.0);
    gsl_matrix * A = gsl_matrix_alloc(6,6);
    gsl_permutation * p = gsl_permutation_alloc(6);
    int status = 0;
    int s;
    double det = 0;
    // recall that the spatial Jacobian Js is in column major.
    for (unsigned int i=0;i<links.size();i++) {
      // compute the determinant of the Jacobian with the ith column removed.
      // since Js is in column major, this is Js with the ith row removed.
      int row = 0;
      for (unsigned int j=0;j<links.size();j++) {
        if (j != i) {
          for (int k=0;k<6;k++) {
            gsl_matrix_set(A, row, k, Js[j][k]);
          }
          row++;
        }
      }
      // take an LU decomposition of A, which is needed to compute the determinant
      status = gsl_linalg_LU_decomp(A, p, &s);
      // find the determinant
      det = gsl_linalg_LU_det(A, s);
      // calculate the nullspace vector
      nullspace[i] = pow(-1.0,i+7)*det;
    }

    // normalize the nullspace vector, if not too close to zero.
    double length = pow(nullspace*nullspace,0.5);
    //std::cout << length << std::endl;
    if (length < 0.01) {
      nullspace.SetAll(0.0);
    } else {
      nullspace = nullspace/length;
    }

    // filter velocity
    vctDynamicVector<double> qdfilt(qd);
    cutoff = 20;
    beta = exp(-1.0*cutoff*dt);
    qdfilt = beta*qdfiltold + (1.0 - beta)*qd;
    qdfiltold = qdfilt;

    // calcuate PD torques associated with nullspace setpoint
    tau_imp_null = -1.0*Kp_null*(q - setpoint_null) - Kd_null*qdfilt;

    // project onto nullspace
    tau_imp_null = (tau_imp_null*nullspace)*nullspace;

    // add in the regular impedance torque
    tau_imp_null += tau_imp;

    // initialize tau_imp_null_fric
    tau_imp_null_fric = tau_imp_null;

    // add in torque due to external impulse. this is for all controllers.
    // specify forces and moments due to external impulse
    vctFixedSizeVector<double,6> FM_external(0.0);
    FM_external[1] = forceExternal[countImpulseSteps];
    // Convert to equivalent wrench in spatial frame
    FM_external[3] = x[1]*FM_external[2] - x[2]*FM_external[1] + FM_external[3];
    FM_external[4] = x[2]*FM_external[0] - x[0]*FM_external[2] + FM_external[4];
    FM_external[5] = x[0]*FM_external[1] - x[1]*FM_external[0] + FM_external[5];
    // Convert FM_external to joint torques
    for (unsigned int i=0;i<links.size();i++) {
      tau_external[i] = 0.0;
      for (int j=0;j<6;j++) {
        tau_external[i] += Js[i][j]*FM_external[j];
      }
    }
    if (fabs(forceExternal[countImpulseSteps]) > 0.001) {
      tau_gc += tau_external;
      tau_ct += tau_external;
      tau_imp += tau_external;
      tau_imp_null += tau_external;
      tau_imp_null_fric += tau_external;
    }


    // for SE3 trajectories:
    // create a potential well to keep the arm inside a specific workspace.
    // for now, I am only putting limits in the y direction because that's
    // all I need for my experiment.
    // IMPORTANT: these limits are hard-coded both here and in the graphics. if you change one, you must go find and change the other.
    const double limitL = -0.2;
    const double limitR = 0.2;
    double dropoff = 0.05;
    vctFixedSizeVector<double,6> FM_wslimits(0.0);
    vctDynamicVector<double> tau_wslimits(7,0.0);

    // scale down controller forces outside workspace limits
    if ((Rtwn[1][3] < limitL) && (FM[1] < 0.0)) {
      FM_wslimits[1] = -1.0*(FM[1] + FM_external[1]);
      if (Rtwn[1][3] > (limitL - dropoff)) {
        //FM_wslimits[1] *= fabs((Rtwn[1][3] - limitL)/dropoff);
        FM_wslimits[1] *= pow((Rtwn[1][3] - limitL)/dropoff,2);
      }
    } else if ((Rtwn[1][3] > limitR) && (FM[1] > 0.0)) {
      FM_wslimits[1] = -1.0*(FM[1] + FM_external[1]);
      if (Rtwn[1][3] < (limitR + dropoff)) {
        //FM_wslimits[1] *= fabs((Rtwn[1][3] - limitR)/dropoff);
        FM_wslimits[1] *= pow((Rtwn[1][3] - limitR)/dropoff,2);
      }
    }
    // create a PD controller pulling in towards the workspace limits
    const double Kp_wslimits = 500;
    const double Kd_wslimits = 10;
    if (Rtwn[1][3] < limitL) {
      FM_wslimits[1] += -1.0*Kp_wslimits*(Rtwn[1][3] - limitL) - Kd_wslimits*xd[1];
    } else if (Rtwn[1][3] > limitR) {
      FM_wslimits[1] += -1.0*Kp_wslimits*(Rtwn[1][3] - limitR) - Kd_wslimits*xd[1];
    }

    // Convert to equivalent wrench in spatial frame
    FM_wslimits[3] = x[1]*FM_wslimits[2] - x[2]*FM_wslimits[1] + FM_wslimits[3];
    FM_wslimits[4] = x[2]*FM_wslimits[0] - x[0]*FM_wslimits[2] + FM_wslimits[4];
    FM_wslimits[5] = x[0]*FM_wslimits[1] - x[1]*FM_wslimits[0] + FM_wslimits[5];
    // Convert FM_wslimits to joint torques
    for (unsigned int i=0;i<links.size();i++) {
      tau_wslimits[i] = 0.0;
      for (int j=0;j<6;j++) {
        tau_wslimits[i] += Js[i][j]*FM_wslimits[j];
      }
    }
    tau_imp += tau_wslimits;
    tau_imp_null += tau_wslimits;
    tau_imp_null_fric += tau_wslimits;

    /*
    if ((controller == impedance_SE3) || (controller == impedance_SE3_null) || (controller == impedance_SE3_null_fric)) {
      std::cout << Rtwn[1][3] << "\t" << FM_wslimits[1] << "\t" << FM[1] << std::endl << tau_wslimits << std::endl << tau << std::endl << std::endl;
    }
    */

    // add in friction compensation, if applicable.
    // this is only for the controller impedanceSE3_null_fric.
    // only apply friction torque if the controller force
    // (FM[1] + FM_external[1] + FM_wslimits[1]) is nonzero.
    if (fabs(FM[1] + FM_external[1] + FM_wslimits[1]) > 0.01) {

      vctFixedSizeMatrix<double,7,2> coulomb; // parameters for coulomb friction (positive vel, then negative vel)
      coulomb[0][0] = 1.3868;
      coulomb[0][1] = 2.6014;
      coulomb[0][2] = 0.6413;
      coulomb[0][3] = 1.3600;
      coulomb[0][4] = 0;
      coulomb[0][5] = 0.5475;
      coulomb[0][6] = 0.0586;
      coulomb[1][0] = 3.7397;
      coulomb[1][1] = 2.0827;
      coulomb[1][2] = 0.9473;
      coulomb[1][3] = 1.1622;
      coulomb[1][4] = 0;
      coulomb[1][5] = 0.2769;
      coulomb[1][6] = 0.0493;

      // need qsd because it is less noisy than qd or qdfilt.

      // first get the vws vector (MLS notation) from xds, augmented with 0 desired nullspace velocity.
      vctFixedSizeVector<double,6> vws(0.0);
      gsl_vector * vws_gsl = gsl_vector_alloc(7);
      gsl_vector * qsd_gsl = gsl_vector_alloc(7);
      vws[3] = xds[3];
      vws[4] = xds[4];
      vws[5] = xds[5];
      vws[0] = xds[0] - (vws[4]*x[2] - vws[5]*x[1]);
      vws[1] = xds[1] - (vws[5]*x[0] - vws[3]*x[2]);
      vws[2] = xds[2] - (vws[3]*x[1] - vws[4]*x[0]);
      for (int i=0;i<6;i++) {
        gsl_vector_set(vws_gsl, i, vws[i]);
      }
      gsl_vector_set(vws_gsl, 6, 0.0);

      gsl_matrix * Jsa = gsl_matrix_alloc(7,7);
      gsl_permutation * pa = gsl_permutation_alloc(7);

      // get the augmented Jacobian from the nullspace calculation
      for (int i=0;i<7;i++) {
        for (int j=0;j<6;j++) {
          gsl_matrix_set(Jsa, j, i, Js[i][j]);
        }
        gsl_matrix_set(Jsa, 6, i, nullspace[i]);
      }

      // solve for qsd
      // take an LU decomposition of Jsa
      status = gsl_linalg_LU_decomp(Jsa, pa, &s);
      // check the determinant; if Jsa is singular, just set qsd to zero instead of solving.
      det = gsl_linalg_LU_det(Jsa, s);
      if (fabs(det) < 0.01) {
        qsd.SetAll(0.0);
      } else {
        // solve for qsd in vws = Jsa*qsd
        status = gsl_linalg_LU_solve(Jsa, pa, vws_gsl, qsd_gsl);
        // copy it back to the cisst vector
        for (int i=0;i<7;i++) {
          qsd[i] = gsl_vector_get(qsd_gsl, i);
        }
      }

      double alpha = 50.0;
      for (int i=0;i<7;i++) {
        if (qsd[i] > 0.0) {
          tau_imp_null_fric[i] += 2.0*friction_gain*coulomb[0][i]/3.14159*atan(alpha*qsd[i]);
        } else {
          tau_imp_null_fric[i] += 2.0*friction_gain*coulomb[1][i]/3.14159*atan(alpha*qsd[i]);
        }
      }
    }


    // if controller type just switched, ramp from old controller to new controller. otherwise,
    // just use current controller.
    double multiplier = 1.0;
    if (newController) {
      if ((t - t_transition) < transitionTime) {
        multiplier = (t - t_transition)/transitionTime;
        tau = (1.0 - multiplier)*tau_transition;
      } else {
        newController = false;
        ROS_INFO("Controller transition complete.");
      }
    }

    if (controller == gravityCompensation) {
      tau += multiplier*tau_gc;
    } else if (controller == computedTorque_Rn) {
      tau += multiplier*tau_ct;
    } else if (controller ==  impedance_SE3) {
      tau += multiplier*tau_imp;
    } else if (controller ==  impedance_SE3_null) {
      tau += multiplier*tau_imp_null;
    } else if (controller ==  impedance_SE3_null_fric) {
      tau += multiplier*tau_imp_null_fric;
    }

    // apply torque saturation limits. to keep arm movement in the same direction, scale
    // the whole torque vector equally so that the joint that is farthest over its limit
    // ends up equal to its limit.
    double saturation_factor = 1.0;
    for (unsigned int i=0;i<links.size();i++) {
      if (tau[i] > saturation_limits[i]) {
        std::cout << "WARNING: " << name << ": Joint " << i << " reached saturation limit. tau = " << tau[i] << ", limit = " << saturation_limits[i] << std::endl;
        if ((tau[i]/saturation_limits[i]) > saturation_factor) {
          saturation_factor = tau[i]/saturation_limits[i];
        }
      }
    }
    tau = tau/saturation_factor;

    xdold = xd;
    xold = x;
  }

  tauold = tau;
  qold = q;
  qdold = qd;
  told = t;
  told_fb = t_fb;

  output->SetForceTorque( tau );

  ControllerData data;
  data.t = t;
  data.dt = dt;
  data.q = (mtsVector<double>)q;
  data.qs = (mtsVector<double>)qs;
  data.qd = (mtsVector<double>)qd;
  data.qsd = (mtsVector<double>)qsd;
  data.setpoint_null = (mtsVector<double>)setpoint_null;
  data.tau_gc = (mtsVector<double>)tau_gc;
  data.tau_ct = (mtsVector<double>)tau_ct;
  data.tau_imp = (mtsVector<double>)tau_imp;
  data.tau_imp_null = (mtsVector<double>)tau_imp_null;
  data.tau_imp_null_fric = (mtsVector<double>)tau_imp_null_fric;
  data.tau_external = (mtsVector<double>)tau_external;
  data.tau = (mtsVector<double>)tau;
  data.controllerNumber = (int)controller;
  data.theta = theta;
  data.m_y = M[1][1];

  for (int i=0;i<7;i++) {
    for (int j=0;j<7;j++) {
      if ((i<3) && (j<3)) {
        data.M[i][j] = M[i][j];
      }
      if ((i<6) && (j<6)) {
        data.M_OS[i][j] = M_OS[i][j];
      }
      data.M_JS[i][j] = M_JS[i][j];
    }
  }

  for (int i=0;i<7;i++) {
    if (i < 3) {
      data.erroraxis[i] = erroraxis[i];
    }
    if (i < 6) {
      data.xds[i] = xds[i];
      data.Kp_SE3[i] = Kp_SE3[i][i];
      data.Kd_SE3[i] = Kd_SE3[i][i];
    }
    data.Kp_Rn[i] = Kp_Rn[i][i];
    data.Kd_Rn[i] = Kd_Rn[i][i];
  }
  data.Kp_null = Kp_null;
  data.Kd_null = Kd_null;
  data.friction_gain = friction_gain;
  data.impulse = impulse;
  data.forceExternal = forceExternal[countImpulseSteps];
  for (int row=0;row<4;row++) {
    for (int col=0;col<4;col++) {
      data.Rtwn[row][col] = Rtwn[row][col];
      data.Rtwns[row][col] = Rtwns[row][col];
    }
  }
  data.tracking_goal = tracking_goal;
  PublishControllerData(data);

  forceExternal[countImpulseSteps] = 0.0;
  countImpulseSteps++;
  if (countImpulseSteps > nImpulseSteps) {
    countImpulseSteps = 0;
  }
}

