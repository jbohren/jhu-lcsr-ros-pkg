#include <cisstRobot/robLinearSE3.h>
#include "wam_impedance_experiment/trajectories/devVelEqn.h"

#include "wam_impedance_experiment/devices/devDAS6014.h"

#include <cisstMultiTask/mtsInterfaceRequired.h>

const std::string devVelEqn::WamStateInput = "WamStateInput";

devVelEqn::devVelEqn( const std::string& name, 
			    double period, 
			    devTrajectory::State state,
			    osaCPUMask cpumask,
			    const std::string& robfile,
			    const vctFrame4x4<double>& Rtw0,
			    devTrajectory::Variables variables,
			    const vctFrame4x4<double>& Rtinit ) :

  devTrajectory( name, period, state, cpumask, devTrajectory::TRACK ),
  robManipulator_AAB( robfile, Rtw0 ),
  input( NULL ),
  output( NULL ),
  Rtinit( Rtinit ),
  Rtold( Rtinit ),
  Rtp( Rtinit ),
  Rtn( Rtinit ),
  firstTime( true ),
  outOfRange( false ),
  t_start( 0.0 ),
  told( 0.0 ),
  t1old( 0.0 ),
  phase( 0.0 ),
  count ( 1 ),
  F( 0.0 ),
  Fold( 0.0 ){

  // Output to the WAM: position and velocity in SE3
  output = RequireOutputSE3( devTrajectory::Output, variables );

  // Input from the arm 
  wam_state = RequireInputRn( devVelEqn::WamStateInput, variables, links.size());

  // Set the output in case the trajectory is started after other components
  vw.SetAll(0.0);
  vdwd.SetAll(0.0);
  output->SetPosition( Rtold );
  output->SetVelocity( vw );
  output->SetAcceleration( vdwd );


}

//void devVelEqn::Startup(){}

// always new
bool devVelEqn::IsInputNew(){ 
  return true; 
}

robFunction* devVelEqn::Queue( double t, robFunction* function )
{ return Track( t, function ); }

robFunction* devVelEqn::Track( double t1, robFunction* ){

  // IN: time, robFunction
  // OUT: robFunction (requires old pose, desired pose, linear velocity, angular velocity, time)
  //
  // calculate deltat
  // check for trajectory enabled
  //  if deltat large, trajectory was just enabled
  //   indicate firstTime
  //   reset counter
  //  if deltat small, trajectory has been running
  // check for firstTime (this is the actual first time, or any time the trajectory is re-enabled)
  //  if firstTime
  //   set deltat = 0
  //   set start time
  //   turn off firstTime flag
  //   check actual position
  //    if inside sine wave range, calculate phase
  //    if outside sine wave range, set outOfRange flag
  //   write old position equal to actual current position
  //  else
  //   write old position - must be calculated from previous old position and previous commanded velocity, and deltat
  // get new desired velocity, position
  //  if outOfRange
  //   previous step was out of range, check for in range now (using commanded position, not actual)
  //    if now in range
  //     clear outOfRange flag
  //     set firstTime
  //     set desired velocity 0, desired position = Rtold (i.e., don't move)
  //    else - still outside of range, use max velocity
  //     if above range, use max negative velocity; set vel = 0.4, Rt = Rtn
  //     if below range, use max positive velocity; set vel = 0.4, Rt = Rtp
  //  else - inside sine wave range, use velocity equation (requires prior calculation of phase)
  //   choose desired position
  //    if desired velocity is positive, use Rtp
  //    if desired velocity is negative, use Rtn
  //    if desired velocity is zero, use Rtold
  //   get velocity magnitude to pass to the robFunction
  // create new robFunction and return


  vctDynamicVector<double> q(links.size(), 0.0);
  vctFrame4x4<double> Rttemp( Rtinit );
  double t;

  // Compare current and previous timestep. If difference is large, trajectory was
  // probably just enabled. In that case, we set firstTime back to true to restart the
  // trajectory, and find the necessary phase shift to keep the sine wave centered at zero.
  double deltat = t1 - t1old;
  double y = 0.0;
  if (deltat > 0.1) {
    firstTime = true;
    count = 1;
  }
  // TODO: Each time the trajectory is re-enabled, start writing to a different log file?
  // (How will I know what to name the file?)

  if (firstTime) {
    // on the first timestep, get the current wam position and calculate the phase shift
    // associated with starting at the current position.
    // get current wam position in joint space
    deltat = 0;
    t_start = t1;
    firstTime = false;
    wam_state->GetPosition( q, t ); // this t may not equal t1 in general, but it is the most recent q reading we have
    // get initial pose using forward kinematics
    Rttemp = ForwardKinematics(q);
    // for now, I want to force the orientation in Rtinit, so just take the initial position
    for (int i=0;i<3;i++) {
      Rtold[i][3] = Rttemp[i][3];
    }
    
    // Set two dummy positions well outside the workspace.
    // These will be used with positive and negative velocities as endpoints of interpolation.
    // Velocity specified by the equation will be used as "max velocity" for interpolation.
    // Thus, the trajectory will move at the specified velocity in that direction until the
    // velocity changes (or joint limits are hit).
    Rtp = Rtold;
    Rtp[1][3] = 10.0;
    Rtn = Rtold;
    Rtn[1][3] = -10.0;

    // get cartesian position from forward kinematics
    vctFixedSizeVector<double,3> position = Rtold.Translation();
    y = position[1];
    if (fabs(y-0.1) > 0.5) {
      outOfRange = true;
    } else {
      phase = acos(2.0*(y-0.1));
    }
  } else {
    // get current commanded position from old position, old velocity, and delta-t.
    // store it in Rtold, because it's about to become the old commanded position.
    Rtold[1][3] = Rtold[1][3] + vw[1]*deltat;
  }

  // now get new velocity from velocity equation
  vw.SetAll(0.0);
  vctFrame4x4<double> Rt( Rtold );
  double vel = 0.0;
  // specify velocity with equation
  // first, check if the wam is actually in the range of the sine wave (right now, the amplitude is 0.5)
  if (outOfRange) {
    // outside range of sine wave, so just move towards the range of the sine wave
    // need to check if we've entered the range
    wam_state->GetPosition( q, t );
    Rttemp = ForwardKinematics(q);
    vctFixedSizeVector<double,3> position = Rttemp.Translation();
    y = position[1];
    if (fabs(y-0.1) < 0.5) {
      // just entered range of sine wave. do not move. reset.
      outOfRange = false;
      firstTime = true;
    } else {
      // still outside range of sine wave. continue moving toward desired range.
      if ((y-0.1) > 0.5) {
        vw[1] = -0.4;
        Rt = Rtn;
      } else {
        vw[1] = 0.4;
        Rt = Rtp;
      }
      vel = 0.4;
    }
  } else {
    // inside the range of the sine wave, so use the sine wave
    // calculate velocity from the sine wave equation
    vw[1] = -0.4*sin(0.8*(t1 - t_start) + phase);
    if (vw[1] > 0.0) {
      Rt = Rtp;
      vel = vw[1];
    } else if (vw[1] < 0.0) {
      Rt = Rtn;
      vel = -1.0*vw[1];
    } // else leave vel set to 0.0, to maintain position
    
  }
  
  // output for Matlab
  /*
  std::cout << "y(" << count << ") = " << y << ";" << std::endl;
  std::cout << "ys(" << count << ") = " << Rtold[1][3] << ";" << std::endl;
  std::cout << "t(" << count << ") = " << (t1 - t_start) << ";" << std::endl;
  std::cout << "yd(" << count << ") = " << vw[1] << ";" << std::endl;
  */

  t1old = t1;
  count++;

  // don't know what the new position will be, because we don't know what delta-t will be! 
  // so I can't store the "current" position as "old" now, because I don't know what it is.
  // do it at the beginning of the next function call instead.

  // create a new interpolation
  robLinearSE3* fn = new robLinearSE3( Rtold, Rt, vel, 0.01, t1 ); // 0.0 to command no angular velocity
  return fn;

}

void devVelEqn::Evaluate( double t, robFunction* function ){
  // We expect the function to be linear
  robLinearSE3* linearse3 = dynamic_cast<robLinearSE3*>( function );

  if( linearse3 != NULL ){

    vctFrame4x4<double> Rtnew;
    vctFixedSizeVector<double,6> vwnew, vdwdnew;

    linearse3->Evaluate( t, Rtnew, vwnew, vdwdnew );

    output->SetPosition( Rtnew );
    output->SetVelocity( vwnew );
    output->SetAcceleration( vdwdnew );

    told = t;

  }
}

