#include <cisstRobot/robLinearSE3.h>
#include "wam_impedance_experiment/trajectories/devReturnToZero.h"

#include "wam_impedance_experiment/devices/devDAS6014.h"

#include <cisstMultiTask/mtsInterfaceRequired.h>

const std::string devReturnToZero::WamStateInput = "WamStateInput";

devReturnToZero::devReturnToZero( const std::string& name, 
			    double period, 
			    devTrajectory::State state,
			    osaCPUMask cpumask,
			    const std::string& robfile,
			    const vctFrame4x4<double>& Rtw0,
			    devTrajectory::Variables variables,
			    const vctFrame4x4<double>& Rthome ) :

  devTrajectory( name, period, state, cpumask, devTrajectory::TRACK ),
  robManipulator_AAB( robfile, Rtw0 ),
  input( NULL ),
  output( NULL ),
  Rthome( Rthome ),
  Rtold( Rthome ),
  firstTime( true ),
  told( 0.0 ),
  t1old( 0.0 ),
  count ( 1 ){

  // Output to the WAM: position and velocity in SE3
  output = RequireOutputSE3( devTrajectory::Output, variables );

  // Input from the arm 
  wam_state = RequireInputRn( devReturnToZero::WamStateInput, variables, links.size());

  // Set the output in case the trajectory is started after other components
  vw.SetAll(0.0);
  vdwd.SetAll(0.0);
  output->SetPosition( Rtold );
  output->SetVelocity( vw );
  output->SetAcceleration( vdwd );


}

//void devReturnToZero::Startup(){}

// always new
bool devReturnToZero::IsInputNew(){ 
  return true; 
}

robFunction* devReturnToZero::Queue( double t, robFunction* function )
{ return Track( t, function ); }

robFunction* devReturnToZero::Track( double t1, robFunction* ){

  vctDynamicVector<double> q(links.size(), 0.0);
  vctFrame4x4<double> Rttemp( Rthome );
  double t;

  // Compare current and previous timestep. If difference is large, trajectory was
  // probably just enabled. In that case, set the translation part of Rtold to the
  // current wam position. The orientation is assumed to already be the desired
  // orientation, so the commanded orientation will be the desired orientation from
  // Rthome.
  double deltat = t1 - t1old;
  if (firstTime || (deltat > 0.1)) {
    // get current wam position in joint space
    deltat = 0;
    firstTime = false;
    wam_state->GetPosition( q, t ); // this t may not equal t1 in general, but it is the most recent q reading we have
    // get initial pose using forward kinematics
    Rtold = ForwardKinematics(q);
    // get cartesian position from forward kinematics
    vctFixedSizeVector<double,3> position = Rtold.Translation();
    // set Rtold to desired orientation and current actual position
    Rtold = Rthome;
    for (int i=0;i<3;i++) {
      Rtold[i][3] = position[i];
    }
  } else {
    // get current commanded position from old position, old velocity, and delta-t.
    // store it in Rtold, because it's about to become the old commanded position.
    // if this would put us past Rthome, set Rtold equal to Rthome, because we are
    // done!
    for (int i=0;i<3;i++) {
      // check if home position is between old and new positions
      double oldposition = Rtold[i][3];
      double newposition = Rtold[i][3] + vw[i]*deltat;
      if (((Rthome[i][3] > oldposition) && (Rthome[i][3] < newposition)) || ((Rthome[i][3] < oldposition) && (Rthome[i][3] > newposition))) {
        //home position is between old and new, so we would stop at home instead of moving on to new.
        Rtold[i][3] = Rthome[i][3];
      } else {
        Rtold[i][3] = newposition;
      }
    }
  }

  // now get new velocity
  vw.SetAll(0.0);
  // get difference between home position and current commanded position
  vctFixedSizeVector<double,3> e = Rthome.Translation() - Rtold.Translation();
  double length = pow(e[0],2) + pow(e[1],2) + pow(e[2],2);
  length = pow(length,0.5);
  vctFixedSizeVector<double,3> vel( 0.0 );
  if (length > 0.1) {
    vel = 0.4*e.NormalizedSelf();
  } else {
    vel = e;
  }
  for (int i=0;i<3;i++) {
    vw[i] = vel[i];
  }

  t1old = t1;

  // don't know what the new position will be, because we don't know what delta-t will be! 
  // so I can't store the "current" position as "old" now, because I don't know what it is.
  // do it at the beginning of the next function call instead.

  // create a new interpolation
  robLinearSE3* fn = new robLinearSE3( Rtold, Rthome, 0.4, 0.01, t1 ); // 0.0 to command no angular velocity
  return fn;

}

void devReturnToZero::Evaluate( double t, robFunction* function ){
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

