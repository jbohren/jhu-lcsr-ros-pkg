#include <cisstRobot/robLinearSE3.h>
#include <cisstRobot/robLinearRn.h>
#include "wam_impedance_experiment/trajectories/devMultipleTrajectories.h"

#include <cisstMultiTask/mtsInterfaceProvided.h>
#include <cisstMultiTask/mtsInterfaceRequired.h>
#include <cisstOSAbstraction/osaGetTime.h>

#include "ros/ros.h"

CMN_IMPLEMENT_SERVICES(TrajInfo);

devMultipleTrajectories::devMultipleTrajectories( const std::string& name, 
			    double period, 
			    devTrajectory::State state,
			    osaCPUMask cpumask,
			    const std::string& robfile,
			    const vctFrame4x4<double>& Rtw0,
			    devTrajectory::Variables variables,
          const vctDynamicVector<double>& qhome,
          const bool useActualPosition,
          const double limitL,
          const double limitR,
          const double thetadmax,
          const double vmax,
          const double wmax) :

  devTrajectory( name, period, state, cpumask, devTrajectory::TRACK ),
  robManipulator_AAB( robfile, Rtw0 ),
  outputSE3( NULL ),
  outputRn( NULL ),
  wam_feedback( NULL ),
  trajectory(stationary),
  direction(0),
  newTrajectory(false),
  newSetpoint(false),
  userCommandVelocity(0.0),
  maxCommandSpeed(false),
  qold( qhome ),
  useActualPosition( useActualPosition ),
  limitL(limitL),
  limitR(limitR),
  thetadmax(thetadmax),
  vmax(vmax),
  wmax(wmax),
  told( 0.0 ),
  t_start( 0.0 ),
  admittance(0.2),
  deadband(0.06) {

  Rtold = ForwardKinematics(qhome);
  setpointRn = qold;
  setpointSE3 = Rtold;
  savedPositionRn = qold;
  savedPositionSE3 = Rtold;

  // Output to the controller
  //    position and velocity in SE3
  //    jointspace position and velocity in Rn
  outputSE3 = ProvideOutputSE3( "OutputSE3", variables );
  outputRn = ProvideOutputRn( "OutputRn", variables, qhome.size() );

  // Input from the WAM
  wam_feedback = RequireInputRn( "WAM_Feedback", devRobotComponent::POSITION, links.size() );

  // add interface for switching trajectory generation equations
  mtsInterfaceProvided* trajectory_selection_interface = AddInterfaceProvided( "TrajectoryInfo" );
  if (trajectory_selection_interface) {
    trajectory_selection_interface->AddCommandWrite( &devMultipleTrajectories::SetTrajectoryInfo,
                                                     this,
                                                     "TrajectoryInfoCommand" );
  }

  // add interface for handling user input
  mtsInterfaceProvided* user_input_interface = AddInterfaceProvided( "UserInput" );
  if (user_input_interface) {
    user_input_interface->AddCommandWrite( &devMultipleTrajectories::HandleUserInput,
                                           this,
                                           "UserInputCommand" );
  }

  // add interface for publishing commanded trajectory. this is a required interface to force publishing.
  mtsInterfaceRequired* trajectory_publisher_interface = AddInterfaceRequired( "TrajectoryPublisher" );
  if (trajectory_publisher_interface) {
    trajectory_publisher_interface->AddFunction( "TrajectoryPublishFunction",
                                                  PublishTrajectory );
  }

  // Set the output in case the trajectory is started after other components

  vw.SetAll(0.0);
  vdwd.SetAll(0.0);
  outputSE3->SetPosition( Rtold );
  outputSE3->SetVelocity( vw );
  outputSE3->SetAcceleration( vdwd );

  qd.SetSize(qold.size());
  qd.SetAll(0.0);
  qdd.SetSize(qold.size());
  qdd.SetAll(0.0);
  outputRn->SetPosition( qold );
  outputRn->SetVelocity( qd );
  outputRn->SetAcceleration( qdd );

}


void devMultipleTrajectories::SetTrajectoryInfo (const TrajInfo& data) {

  // check for change in trajectory type.
  //
  // depending on the type of switch, different things need to be done to 
  // transition between trajectory types.
  //
  // Rn to Rn: nothing (new trajectory starts at most recent command from old trajectory)
  // Rn to SE3: nothing (Rtold is kept up to date with forward kinematics in Evaluate)
  // SE3 to SE3: nothing (new trajectory starts at most recent command from old trajectory)
  // SE3 to Rn: if possible, use inverse kinematics to translate most recent SE3 command 
  //            to Rn. if inverse kinematics fail, use either actual robot position or most
  //            recent value of qold. the flag useActualPosition specifies whether to use
  //            the actual position or the inverse kinematics.
  //            note that even though qold is kept up to date with inverse kinematics in
  //            Evaluate, we don't know if the inverse kinematics succeeded or failed, so
  //            we want to check again here.
  //
  // Rn trajectories: setpointsRn
  // SE3 trajectories: userInput, sinusoid, setpointsSE3
  //
  // if switching to stationary, do nothing because we want to keep both current setpoints,
  // whatever they are.

  trajectoryType nextTrajectory = (trajectoryType)(data.trajectoryNumber);

  if (trajectory != nextTrajectory) {

    // check type of switch, and make adjustments as described above
    if ((trajectory == userInput) || (trajectory == sinusoid) || (trajectory == setpointsSE3)) {
      if (nextTrajectory == setpointsRn) {
        // SE3 to Rn switch
        // try inverse kinematics - if success, there's no need to do anything because
        // the last value calculated in Evaluate is up to date. if failure, use actual
        // robot position if useActualPosition flag is set (presumably when we're using
        // the actual robot or a good simulation or it) or just use the bad old value
        // otherwise (presumably when we're using the stubs).
        vctDynamicVector<double> qnew(qold);
        robManipulator_AAB::Errno manerrno;
        manerrno = InverseKinematics(qnew,Rtold,1e-12,100);
        if (manerrno == robManipulator_AAB::EFAILURE) {
          // failure - use current actual position or last good value for qold
          ROS_ERROR_STREAM("Inverse kinematics failure!\nRt = \n" << Rtold << "\nq = " << qnew);
          if (useActualPosition) {
            // get actual robot position
            double t;
            wam_feedback->GetPosition(qold,t); 
          }
        }
      }
    }

    trajectory = (trajectoryType)(data.trajectoryNumber);

    if (trajectory == sinusoid) {
      direction = data.direction;
    } else if (trajectory == userInput) {

      // get admittance and deadband values from parameter server
      
      if (ros::param::get("admittance", admittance)) {
        ROS_INFO_STREAM("Got admittance value " << admittance << " from parameter server.");
      } else {
        ROS_WARN_STREAM("Failed to get admittance value from parameter server. Using previous value, " << admittance << ".");
      }

      if (ros::param::get("deadband", deadband)) {
        ROS_INFO_STREAM("Got deadband value " << deadband << " from parameter server.");
      } else {
        ROS_WARN_STREAM("Failed to get deadband value from parameter server. Using previous value, " << deadband << ".");
      }

    }


    newTrajectory = true;
  }

  /*
  if (useActualPosition && (trajectory == stationary)) {
    // get actual robot position
    double t;
    wam_feedback->GetPosition(qold,t); 
    Rtold = ForwardKinematics(qold);
  }
  */

  if ((trajectory == setpointsRn) && ((setpointRn != data.setpointRn) || newTrajectory)) {
    setpointRn = data.setpointRn;
    newSetpoint = true;
  }

  if ((trajectory == setpointsSE3) && ((setpointSE3 != data.setpointSE3) || newTrajectory)) {
    setpointSE3 = data.setpointSE3;
    newSetpoint = true;
  }

}

void devMultipleTrajectories::HandleUserInput(const double& value) {
  
  // set desired velocity equal to admittance times user input

  ROS_DEBUG("Received user input %f",value);

  double max = 0.5; // maximum command speed

  if (value > deadband) {
    userCommandVelocity = admittance*(value - deadband);
  } else if (value < -1.0*deadband) {
    userCommandVelocity = admittance*(value + deadband);
  } else {
    userCommandVelocity = 0;
  }

  if (fabs(userCommandVelocity) > max) {
    userCommandVelocity = max*userCommandVelocity/fabs(userCommandVelocity);
    if (trajectory == userInput) {
      maxCommandSpeed = true;
    } else {
      maxCommandSpeed = false;
    }
  } else {
    maxCommandSpeed = false;
  }
}

//void devMultipleTrajectories::Startup(){}

bool devMultipleTrajectories::IsInputNew(){ 

  // for trajectories involving setpoints, return true if there
  // is a new setpoint, so Track or Queue gets called.
  // for other trajectories, return false, to skip Track/Queue.

  switch (trajectory) {

    case setpointsRn:
    case setpointsSE3:
      // return true if there is a new setpoint, false otherwise
      if (newSetpoint) { 
        newSetpoint = false;
        return true;
      } else { 
        return false;
      }

    case stationary:
    case userInput:
    case sinusoid:
      // for user input or sinusoid, prefer to short circuit the tracking and queueing functions
      // to define velocity and position at each time step.
      // for stationary, no need for tracking or queueing functions.
      return false; 

    default:
      return false;
  }

}

robFunction* devMultipleTrajectories::Queue( double t, robFunction* function ){

  // create a new interpolation

  if (trajectory == setpointsRn) {

    // previous and next functions
    robLinearRn* previous = dynamic_cast<robLinearRn*>( function );
    robLinearRn* next = NULL;

    // previous and next positions
    vctDynamicVector<double> q1( qold ); // initialize to last commanded position
    vctDynamicVector<double> q2( setpointRn ); // new setpoint
    vctDynamicVector<double> qdmax(7, thetadmax); // max joint velocities

    if( previous != NULL ){

      // Get the final position of the previous function
      vctDynamicVector<double> q1d, q1dd;
      previous->FinalState( q1, q1d, q1dd );

      // Make the next function
      if( previous->StopTime() < t ) {
        // if the previous function has already ended, start the new function now
        next = new robLinearRn( q1, q2, qdmax, t ); 
      } else {
        // if the previous function is still going, start the new function when it ends (this is what Blend does)
        vctDynamicVector<double> zeros( qdmax.size(), 0.0 );
        next = new robLinearRn( q1, q2, qdmax );
        previous->Blend( next, qdmax, zeros );
      }
    } else { 
      // there is no previous function, so start the new function now
      next = new robLinearRn( q1, q2, qdmax, t ); 
    }

    return next;

  } else if (trajectory == setpointsSE3) {

    // previous and next functions
    robLinearSE3* previous = dynamic_cast<robLinearSE3*>( function );
    robLinearSE3* next = NULL;

    // previous and next positions
    vctFrame4x4<double> Rt1( Rtold ); // initialize to last commanded position
    vctFrame4x4<double> Rt2 = setpointSE3; // new setpoint

    if( previous != NULL ){

      // Get the final position of the previous function
      vctFixedSizeVector<double,6> vw, vdwd;
      previous->FinalState( Rt1, vw, vdwd );

      // Make the next function
      if( previous->StopTime() < t ) {
        // if the previous function has already ended, start the new function now
        next = new robLinearSE3( Rt1, Rt2, vmax, wmax, t ); 
      } else {
        // if the previous function is still going, start the new function when it ends (this is what Blend does)
        next = new robLinearSE3( Rt1, Rt2, vmax, wmax );
        previous->Blend( next, vmax, wmax );
      }
    } else {
      // there is no previous function, so start the new function now
      next = new robLinearSE3( Rt1, Rt2, vmax, wmax, t ); 
    }

    return next;

  }

  // should never get here...
  return 0;

}

robFunction* devMultipleTrajectories::Track( double t, robFunction* function )
{

  if (newTrajectory) {

    // If trajectory type just changed, do Track because we're changing function 
    // types, so we want to switch over right away. Otherwise, call Queue to
    // start the next function after the current function ends.

    newTrajectory = false;

    if (trajectory == setpointsRn) {

      robLinearRn* next = NULL;

      // previous and next positions
      vctDynamicVector<double> q1( qold ); // last commanded position
      vctDynamicVector<double> q2( setpointRn ); // new setpoint
      vctDynamicVector<double> qdmax(7, thetadmax); // max joint velocities

      next = new robLinearRn( q1, q2, qdmax, t );

      return next;

    } else if (trajectory == setpointsSE3) {

      robLinearSE3* next = NULL;

      // previous and next positions
      vctFrame4x4<double> Rt1( Rtold ); // last commanded position
      vctFrame4x4<double> Rt2( setpointSE3 ); // new setpoint

      next = new robLinearSE3( Rt1, Rt2, vmax, wmax, t );

      return next;

    }

  } else {

    return( Queue(t, function) );

  }

  // should never get here...
  return 0;
 
}

void devMultipleTrajectories::Evaluate( double t, robFunction* function ){

  vctFrame4x4<double> Rtnew(Rtold);
  vctFixedSizeVector<double,6> vwnew(0.0), vdwdnew(0.0);
  vctDynamicVector<double> qnew(qold), qdnew, qddnew;
  qdnew.SetSize(qold.size());
  qdnew.SetAll(0.0);
  qddnew.SetSize(qold.size());
  qddnew.SetAll(0.0);

  robLinearRn* linearrn;
  robLinearSE3* linearse3;

  // for sinusoid trajectory
  const double multiplier = .18/1.4;
  const double amp1 = multiplier*0.8;
  const double amp2 = multiplier*-0.6;
  const double freq1 = 1.2;
  const double freq2 = 1.9;

  double dropoff = 0.05; // for userInput trajectory near workspace limits

  switch (trajectory) {

    robManipulator_AAB::Errno manerrno;

    case setpointsRn:
    // current function should be in Rn. evaluate at current time.
    linearrn = dynamic_cast<robLinearRn*>( function );
    if( linearrn != NULL ){
      linearrn->Evaluate( t, qnew, qdnew, qddnew );
    }
    // update SE3 setpoint with forward kinematics
    Rtnew = ForwardKinematics(qnew);
    break;

    case setpointsSE3:
    // current function should be in SE3. evaluate at current time.
    linearse3 = dynamic_cast<robLinearSE3*>( function );
    if( linearse3 != NULL ){
      linearse3->Evaluate( t, Rtnew, vwnew, vdwdnew );
    }
    // update Rn setpoint with inverse kinematics, if possible
    manerrno = InverseKinematics(qnew,Rtnew,1e-12,20);
    if (manerrno == robManipulator_AAB::EFAILURE) {
      //ROS_ERROR_STREAM("Inverse kinematics failure!\nRt = \n" << Rtnew << "\nq = " << qnew);
      qnew = qold;
    }
    break;

    case userInput:

    if (newTrajectory) {
      newTrajectory = false;
    }

    // calculate new position and velocity from user input.
    // commanded velocity drops off outside workspace limits.
    vwnew[1] = userCommandVelocity;
    if ((vwnew[1] > 0) && (Rtold[1][3] >= limitR)) {
      // velocity drops off as a quadratic
      if (Rtold[1][3] < (limitR + dropoff)) {
        vwnew[1] *= 1.0 - pow((Rtold[1][3] - limitR)/dropoff,2);
      } else {
        vwnew[1] = 0.0;
      }
    } else if ((vwnew[1] < 0) && (Rtold[1][3] <= limitL)) {
      // velocity drops off as a quadratic
      if (Rtold[1][3] > (limitL - dropoff)) {
        vwnew[1] *= 1.0 - pow((Rtold[1][3] - limitL)/dropoff,2);
      } else {
        vwnew[1] = 0.0;
      }
    }

    // integrate to get y position
    Rtnew[1][3] += vwnew[1]*(t-told);
    // the rest of the SE3 command is unchanged.
    // update Rn setpoint with inverse kinematics, if possible
    manerrno = InverseKinematics(qnew,Rtnew,1e-12,20);
    if (manerrno == robManipulator_AAB::EFAILURE) {
      //ROS_ERROR_STREAM("Inverse kinematics failure!\nRt = \n" << Rtnew << "\nq = " << qnew);
      qnew = qold;
    }

    break;

    case sinusoid:
    if (newTrajectory) {
      if (fabs(Rtnew[1][3]) < 0.005) {
        // close to zero - start sinusoid on next timestep
        t_start = t;
        Rtnew[1][3] = 0.0;
        newTrajectory = false;
      } else {
        // move toward zero at a constant velocity
        if (Rtnew[1][3] > 0.0) {
          vwnew[1] = -1.0*vmax;
        } else {
          vwnew[1] = vmax;
        }
        Rtnew[1][3] += vwnew[1]*(t-told);
      }
    } else {
      // y position from equation
      Rtnew[1][3] = direction*(amp1*sin(freq1*(t - t_start)) + amp2*sin(freq2*(t - t_start)));
      // y velocity from equation
      vwnew[1] = direction*(amp1*freq1*cos(freq1*(t - t_start)) + amp2*freq2*cos(freq2*(t - t_start)));
      // the rest of the SE3 command is unchanged.
    }
    // update Rn setpoint with inverse kinematics, if possible
    manerrno = InverseKinematics(qnew,Rtnew,1e-12,20);
    if (manerrno == robManipulator_AAB::EFAILURE) {
      //ROS_ERROR_STREAM("Inverse kinematics failure!\nRt = \n" << Rtnew << "\nq = " << qnew);
      qnew = qold;
    }
    break;

    case stationary:
    if (newTrajectory) {
      newTrajectory = false;
    }
    // do nothing here, so previous commanded position and velocity remain.
    break;

  }

  outputSE3->SetPosition( Rtnew );
  outputSE3->SetVelocity( vwnew );
  outputSE3->SetAcceleration( vdwdnew );

  outputRn->SetPosition( qnew );
  outputRn->SetVelocity( qdnew );
  outputRn->SetAcceleration( qddnew );

  // publish trajectory output on a ROS topic
  TrajInfo data;
  data.t = osaGetTime();
  data.trajectoryNumber = (int)trajectory;
  data.direction = 0;
  data.setpointRn.SetSize(qnew.size());
  data.setpointRn = qnew;
  data.setpointSE3 = Rtnew;
  data.velocitySE3 = vwnew;
  data.maxCommandSpeed = maxCommandSpeed;
  PublishTrajectory(data);

  told = t;
  qold = qnew;
  Rtold = Rtnew;

}

