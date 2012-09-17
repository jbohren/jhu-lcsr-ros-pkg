/*
 * Impedance control in Cartesian space, world frame with compensation for gravity, coriolis forces, and inertia.
 * Requires inputs of desired position, velocity, acceleration in world frame.
 * Requires Kp, Kd in world frame.
 * Outputs joint torques.
 */

#include <fstream>
#include <iostream>
#include "wam_impedance_experiment/controllers/devImpedance.h"
#include "wam_impedance_experiment/devices/devDAS6014.h"
#include <cisstMultiTask/mtsInterfaceRequired.h>

devImpedance::devImpedance( const std::string& taskname, 
			    double period,
			    devController::State state,
			    osaCPUMask mask,
			    const std::string& robfile,
			    const vctFrame4x4<double>& Rtw0,
			    const vctFixedSizeMatrix<double,6,6>& Kp,
			    const vctFixedSizeMatrix<double,6,6>& Kd,
			    const std::string datafile) :
  
  devController( taskname, period, state, mask ),
  robManipulator_AAB( robfile, Rtw0 ),
  input( NULL ),
  output( NULL ),
  feedback( NULL ),
  Kp(Kp),
  Kd(Kd),
  datafile(datafile),
  told( -1.0 ),
  EnableOutput( false ),
  FirstWrite( true ),
  count(1){

  //eold.SetSize( links.size() );
  eold.SetAll( 0.0 );

  input = ProvideInputSE3( devController::Input,
			  devRobotComponent::POSITION | 
			  devRobotComponent::VELOCITY );

  output = RequireOutputRn( devController::Output,
			    devRobotComponent::FORCETORQUE,
			    links.size() );

  feedback = RequireInputRn( devController::Feedback,
			     devRobotComponent::POSITION,
			     links.size() );

}

void devImpedance::Evaluate(){
  
  double t;
  vctDynamicVector<double> q(links.size(), 0.0);
  vctDynamicVector<double> qd(links.size(), 0.0);
  vctFixedSizeVector<double,6> xd(0.0);
  vctFixedSizeVector<double,6> xd_unfilt(0.0);
  double cutoff = 1000;
  vctFrame4x4<double> Rtwns;
  vctFixedSizeVector<double,6> vw(0.0);     // 6-vector of translational and rotational end effector velocities - 
					    // note that this is the vw vector used in MLS, see pp 54-55 for interpretation.
  vctFixedSizeVector<double,6> xds(0.0);    // 6-vector of desired translational and rotational end effector velocities -
					    // note that this is NOT the vw vector from MLS, this is just world velocities
					    // of the end effector.

  vctFixedSizeVector<double,6> FM(0.0);
  vctDynamicVector<double> tau(links.size(), 0.0);
  vctDynamicVector<double> tauold(links.size(), 0.0);

  input->GetPosition( Rtwns, t );
  input->GetVelocity( xds, t );

  feedback->GetPosition( q, t );
std::cout << q << std::endl;

  vctFixedSizeVector<double,3> xs = Rtwns.Translation();

  double dt = t - told;
  vctFixedSizeVector<double,3> x(0.0);
  vctFrame4x4<double> Rtwn = ForwardKinematics(q); // returns homogeneous transformation from end effector frame to world frame for joint angles q
  // Get actual position of robot in Cartesian space, world frame
  x = Rtwn.Translation();

  if( 0 < dt && 0 < told ){
    qd = (q - qold)/dt;

std::cout << "before Jacobians\n";
    JacobianBody(q); // updates the body Jacobian
std::cout << "between Jacobians\n";
    JacobianSpatial(q); // updates the spatial Jacobian
std::cout << "after Jacobians\n";


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

    xd_unfilt = xd;
    // filter velocity - y-axis angular velocity only
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
    FM = -1.0*Kp*e - Kd*ed;
    
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
      tau[i] = 0.0;
      for (int j=0;j<6;j++) {
        tau[i] += Js[i][j]*FM[j];
      }
    }


    // Add in the coriolis+gravity load

    vctDynamicVector<double> ccg = CCG( q, qd );
    tau = tau + ccg;

    eold = e;

// output for Matlab, if enabled (edit as desired)
    if (EnableOutput) {
      std::ofstream outputfile;
      char filename[20];
      strcpy(filename, datafile.c_str());
      if (FirstWrite) {
        outputfile.open(filename,std::ios::trunc);
        FirstWrite = false;
      } else {
        outputfile.open(filename,std::ios::app);
      }
      int steps = 2;
      if ((count % steps) == 0) {
	outputfile << "t(" << count/steps << ") = " << t << ";" << std::endl;
        outputfile << "q(:," << count/steps << ") = [" << q << "];" << std::endl;
        outputfile << "qd(:," << count/steps << ") = [" << qd << "];" << std::endl;
        outputfile << "x(:," << count/steps << ") = [" << x << "];" << std::endl;
        outputfile << "xd(:," << count/steps << ") = [" << xd << "];" << std::endl;
        outputfile << "xd_raw(:," << count/steps << ") = [" << xd_unfilt << "];" << std::endl;
        outputfile << "xs(:," << count/steps << ") = [" << xs << "];" << std::endl;
        outputfile << "Rtwns{" << count/steps << "} = reshape([" << Rtwns << "],4,4);" << std::endl;
        outputfile << "xds(:," << count/steps << ") = [" << xds << "];" << std::endl;
        outputfile << "vw(:," << count/steps << ") = [" << vw << "];" << std::endl;
        outputfile << "cutoff" << " = " << cutoff << ";" << std::endl;
        outputfile << "FM(:," << count/steps << ") = [" << FM << "];" << std::endl;
        outputfile << "tau(:," << count/steps << ") = [" << tau << "];" << std::endl;
/*
	vctDynamicMatrix<double> M_JS = JSinertia( q );
        double M_OS[6][6];
        OSinertia( M_OS,q ); // Cartesian inertia in the body frame
	// print out inertia
	std::cout << "M_JS{" << count/steps << "} = reshape([" << M_JS << "],7,7);" << std::endl;
	std::cout << "M_OS{" << count/steps << "} = [";
	for (int k=0;k<6;k++) {
	  for (int m=0;m<6;m++) {
	    std::cout << M_OS[k][m] << ",";
	  }
	  std::cout << ";" << std::endl;
	}
	std::cout << "];" << std::endl;
*/
      }
      count++;
      outputfile.close();

      // disable output if we're done
/*
      if (fabs(xs[0] + 0.85) >= 0.001) {
	EnableOutput = false;
      }
    } else {
      // enable output if time is right
      if ((fabs(xs[0] + 0.85) < 0.001) && (fabs(xs[1] + 0.4) < 0.001)) {
	EnableOutput = true;
      }
*/
    }


  } else if (dt == 0) {
    // if we got a bad time reading, apply same torque as previous timestep
    tau = tauold;
  }

  tauold = tau;
  qold = q;
  xdold = xd;
//  qsold = qs;
  told = t;
  xold = x;
//tau[1] += 0.02;
  //	    << "tau: " << tau << std::endl;
  output->SetForceTorque( tau );

}
