#include <cisstRobot/robLinearSE3.h>
#include "wam_impedance_experiment/trajectories/devForceInput.h"

#include "wam_impedance_experiment/devices/devDAS6014.h"

#include <cisstMultiTask/mtsInterfaceRequired.h>

const std::string devForceInput::WamStateInput = "WamStateInput";

devForceInput::devForceInput( const std::string& name, 
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
  told( 0.0 ),
  t2old( 0.0 ),
  count ( 1 ),
  F( 0.0 ),
  Fold( 0.0 ){

  // Output to the WAM: position and velocity in SE3
  output = RequireOutputSE3( devTrajectory::Output, variables );

  // Input from the arm 
  wam_state = RequireInputRn( devForceInput::WamStateInput, variables, links.size());

  // Set the output in case the trajectory is started after other components
  vw.SetAll(0.0);
  vdwd.SetAll(0.0);
  output->SetPosition( Rtold );
  output->SetVelocity( vw );
  output->SetAcceleration( vdwd );

  mtsInterfaceRequired* das6014;
  das6014 = AddInterfaceRequired( "Input" );
  if( das6014 ) {
    das6014->AddFunction( devDAS6014::ReadVolt, ReadVolt );
    das6014->AddFunction( devDAS6014::GetZero,  GetZero );
  }
}

//void devForceInput::Startup(){}

vctFixedSizeVector<double,6> devForceInput::GetInput( double t2 ){
  vctFixedSizeVector<double,6> vw_in(0.0);
  // specify velocity with force sensor input
  const double alpha = 0.05;
  const double deadband = 0.11;
  mtsDouble V, zero;
  mtsInt channel = 0;
  ReadVolt( channel, V );
  GetZero( channel, zero );
  Fold = F; // previous filtered value
  F = ((double)V-(double)zero)*73.7; // new unfiltered value

  // Lowpass filter input
  const double cutoff = 40; // cutoff frequency in radians per second
  double beta = exp(-1*cutoff*(t2-t2old));
  F = beta*Fold + (1.0 - beta)*F; // new filtered value

  if (F > deadband) {
    vw_in[1] = alpha*(F - deadband);
  } else if (F < -1.0*deadband) {
    vw_in[1] = alpha*(F + deadband);
  }
  t2old = t2;
  return vw_in;
}

// always new
bool devForceInput::IsInputNew(){ 
  return true; 
}

robFunction* devForceInput::Queue( double t, robFunction* function )
{ return Track( t, function ); }

robFunction* devForceInput::Track( double t1, robFunction* ){

  vctDynamicVector<double> q(links.size(), 0.0);
  vctFrame4x4<double> Rttemp( Rtinit );
  double t;

  // Compare current and previous timestep. If difference is large, trajectory was
  // probably just enabled. In that case, we set firstTime back to true to restart the
  // trajectory, and find the necessary phase shift to keep the sine wave centered at zero.
  double deltat = t1 - t1old;
  if (deltat > 0.1) {
    firstTime = true;
    count = 1;
  }
  // TODO: Each time the trajectory is re-enabled, start writing to a different log file?
  // (How will I know what to name the file?)

  if (firstTime) {
    // on the first timestep, get the current wam position and make that the start position, instead of Rtinit
    // get current wam position in joint space
    deltat = 0;
    firstTime = false;
    wam_state->GetPosition( q, t ); // this t may not equal t1 in general, but it is the most recent q reading we have
    // get initial pose using forward kinematics
    Rttemp = ForwardKinematics(q);
    // I want to force the orientation in Rtinit, so just take the initial position
    for (int i=0;i<3;i++) {
      Rtold[i][3] = Rttemp[i][3];
    }

    // Set two dummy positions well outside the workspace.
    // These will be used with positive and negative velocities as endpoints of interpolation.
    // Velocity specified by force sensor input will be used as "max velocity" for interpolation.
    // Thus, the trajectory will move at the specified velocity in that direction until the
    // velocity changes (or joint limits are hit).
    Rtp = Rtold;
    Rtp[1][3] = 10.0;
    Rtn = Rtold;
    Rtn[1][3] = -10.0;
  } else {
    // get current commanded position from old position, old velocity, and delta-t.
    // store it in Rtold, because it's about to become the old commanded position.
    Rtold[1][3] = Rtold[1][3] + vw[1]*deltat;
  }

  // now get new velocity from force sensor input (filtered)
  vw = GetInput(t1);

  vctFrame4x4<double> Rt( Rtold );
  double vel = 0.0;
  if (vw[1] > 0.0) {
    Rt = Rtp;
    vel = vw[1];
  } else if (vw[1] < 0.0) {
    Rt = Rtn;
    vel = -1.0*vw[1];
  } // else keep vel at 0.0, to maintain position

  t1old = t1;
  count++;

  // don't know what the new position will be, because we don't know what delta-t will be! 
  // so I can't store the "current" position as "old" now, because I don't know what it is.
  // do it at the beginning of the next function call instead.

  // create a new interpolation
  robLinearSE3* fn = new robLinearSE3( Rtold, Rt, vel, 0.01, t1 );
  return fn;

}

void devForceInput::Evaluate( double t, robFunction* function ){
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

