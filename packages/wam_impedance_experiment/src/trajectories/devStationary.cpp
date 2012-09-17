#include <cisstRobot/robLinearSE3.h>
#include "wam_impedance_experiment/trajectories/devStationary.h"

#include "wam_impedance_experiment/devices/devDAS6014.h"

#include <cisstMultiTask/mtsInterfaceRequired.h>

const std::string devStationary::WamStateInput = "WamStateInput";

devStationary::devStationary( const std::string& name, 
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
  t1old( 0.0 ){

  // Output to the WAM: position and velocity in SE3
  output = RequireOutputSE3( devTrajectory::Output, variables );

  // Input from the arm 
  wam_state = RequireInputRn( devStationary::WamStateInput, variables, links.size());

  // Set the output in case the trajectory is started after other components
  vw.SetAll(0.0);
  vdwd.SetAll(0.0);
  output->SetPosition( Rtold );
  output->SetVelocity( vw );
  output->SetAcceleration( vdwd );


}

// The purpose of this trajectory is just to sit still and not do anything!

//void devStationary::Startup(){}

// always new
bool devStationary::IsInputNew(){ 
  return true; 
}

robFunction* devStationary::Queue( double t, robFunction* function )
{ return Track( t, function ); }

robFunction* devStationary::Track( double t1, robFunction* ){

  vctDynamicVector<double> q(links.size(), 0.0);
  vctFrame4x4<double> Rttemp( Rthome );
  double t;

  // Compare current and previous timestep. If difference is large, trajectory was
  // probably just enabled. In that case, get the current wam pose and set that as
  // the desired pose.
  double deltat = t1 - t1old;
  if (firstTime || (deltat > 0.1)) {
    // get current wam position in joint space
    deltat = 0;
    firstTime = false;
    wam_state->GetPosition( q, t ); // this t may not equal t1 in general, but it is the most recent q reading we have
    // get initial pose using forward kinematics
    Rtold = Rthome;
  }

  // set new velocity
  vw.SetAll(0.0);

  t1old = t1;

  // create a new interpolation
  robLinearSE3* fn = new robLinearSE3( Rtold, Rthome, 0.4, 0.01, t1 ); // 0.0 to command no angular velocity
  return fn;

}

void devStationary::Evaluate( double t, robFunction* function ){
  // We expect the function to be linear
  robLinearSE3* linearse3 = dynamic_cast<robLinearSE3*>( function );

  if( linearse3 != NULL ){

    vctFrame4x4<double> Rtnew;
    vctFixedSizeVector<double,6> vwnew, vdwdnew;

    linearse3->Evaluate( t, Rtnew, vwnew, vdwdnew );

    std::cout << "Rt = " << std::endl << Rtnew << std::endl;
    std::cout << "vw = " << vw << std::endl;

    output->SetPosition( Rtnew );
    output->SetVelocity( vwnew );
    output->SetAcceleration( vdwdnew );

    told = t;

  }
}

