/*
 * Impedance control in Cartesian space, world frame with compensation for gravity, coriolis forces, and inertia.
 * Requires inputs of desired position, velocity, acceleration in world frame.
 * Requires Kp, Kd in world frame.
 * Requires Ki (desired inertia) in world frame.
 * Outputs joint torques.
 */

#include "wam_impedance_experiment/controllers/devImpedanceIC.h"
#include "wam_impedance_experiment/devices/devDAS6014.h"
#include <cisstMultiTask/mtsInterfaceRequired.h>

devImpedanceIC::devImpedanceIC( const std::string& taskname, 
			    double period,
			    devController::State state,
			    osaCPUMask mask,
			    const std::string& robfile,
			    const vctFrame4x4<double>& Rtw0,
			    const vctFixedSizeMatrix<double,6,6>& Ki,
			    const vctFixedSizeMatrix<double,6,6>& Kp,
			    const vctFixedSizeMatrix<double,6,6>& Kd,
			    const std::string datafile ) :
  
  devController( taskname, period, state, mask ),
  robManipulator_AAB( robfile, Rtw0 ),
  input( NULL ),
  output( NULL ),
  feedback( NULL ),
  Ki(Ki),
  Kp(Kp),
  Kd(Kd),
  datafile(datafile),
  told( -1.0 ),
  EnableOutput( true ),
  count( 1 ),
  FirstWrite( true ){

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

void devImpedanceIC::Evaluate(){
  
  double t;
  vctDynamicVector<double> q(links.size(), 0.0);
  vctDynamicVector<double> qd(links.size(), 0.0);
  vctDynamicVector<double> qdd(links.size(), 0.0);
  vctDynamicVector<double> qd_filt(links.size(), 0.0);
  vctDynamicVector<double> qdd_unfilt(links.size(), 0.0);
  vctFixedSizeVector<double,6> xd(0.0);
  vctFixedSizeVector<double,6> xd_filt(0.0);
  vctFrame4x4<double> Rtwns;
  vctFixedSizeVector<double,6> vw(0.0);     // 6-vector of translational and rotational end effector velocities - 
					    // note that this is the vw vector used in MLS, see pp 54-55 for interpretation.
  vctFixedSizeVector<double,6> xds(0.0);    // 6-vector of desired translational and rotational end effector velocities -
					    // note that this is NOT the vw vector from MLS, this is just world velocities
					    // of the end effector.
  vctFixedSizeVector<double,6> xdd(0.0);    // acceleration of the end effector
  vctFixedSizeVector<double,6> xdd_unfilt(0.0);    // acceleration of the end effector
  //vctFixedSizeVector<double,6> xddsb(0.0);  // desired accelerations in body frame, for inertia compensation
  vctFixedSizeVector<double,6> xdsb(0.0);   // desired velocities in body frame, to calculate xddsb

  vctDynamicVector<double> tau(links.size(), 0.0);
  vctDynamicVector<double> tauold(links.size(), 0.0);

  input->GetPosition( Rtwns, t );
  input->GetVelocity( xds, t );

  feedback->GetPosition( q, t );

  vctFixedSizeVector<double,3> xs = Rtwns.Translation();

  double dt = t - told;
  vctFixedSizeVector<double,3> x(0.0);
  vctFrame4x4<double> Rtwn = ForwardKinematics(q); // returns homogeneous transformation from end effector frame to world frame for joint angles q
  // Get actual position of robot in Cartesian space, world frame
  x = Rtwn.Translation();

  // for filtering - will change as needed
  double cutoff = 40;
  double beta = 1;

  if( 0 < dt && 0 < told ){
    qd = (q - qold)/dt;
    // filter qd for use in calculating qdd (use unfiltered to get xd)
    cutoff = 10;
    beta = exp(-1.0*cutoff*dt);
    qd_filt = beta*qd_filt_old + (1.0 - beta)*qd;
    // calculate qdd
    qdd_unfilt = (qd_filt - qd_filt_old)/dt;
    // filter qdd - use this filtered value for inertia compensation
    qdd = beta*qddold + (1.0 - beta)*qdd_unfilt;

    vctFixedSizeVector<double,3> xd3 = (x - xold)/dt;
    vctFixedSizeVector<double,6> xd_unfilt(0.0);

    JacobianBody(q); // updates the body Jacobian
    JacobianSpatial(q); // updates the spatial Jacobian


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
    // filter velocity
    // angular velocity about y axis tends to be noisy, because it's that last joint (in the configuration I'm using)
    // so filter that one more than the rest.
    // I probably should be putting this as a variable matrix of cutoffs set by the main program instead of the controller, but... 
    // Note that right now, xd_filt is filtered so I can differentiate to get xdd.  But for the derivative portion of the control, 
    // I'm only filtering xd[4], angular velocity about the world y axis.
    cutoff = 40;
    beta = exp(-1.0*cutoff*dt);
    xd_filt = beta*xd_filt_old + (1.0 - beta)*xd_unfilt;
    xd_filt[4] = xd_unfilt[4];
    cutoff = 5;
    beta = exp(-1.0*cutoff*dt);
    xd_filt[4] = beta*xd_filt_old[4] + (1.0 - beta)*xd_unfilt[4];
    xd[4] = xd_filt[4];

    // differentiate to get acceleration, then filter
    cutoff = 10;
    beta = exp(-1.0*cutoff*dt);
    xdd_unfilt = (xd_filt - xd_filt_old)/dt;
    xdd = beta*xddold + (1.0 - beta)*xdd_unfilt;

    // error = current - desired
    vctFixedSizeVector<double,3> ex = x - xs;			     // 3-vector for just position error
    vctFixedSizeVector<double,3> ea( 0.0 );			     // 3-vector for just orientation error
    ea[0] = -0.5*( Rtwn[1][0]*Rtwns[2][0] - Rtwn[2][0]*Rtwns[1][0] + Rtwn[1][1]*Rtwns[2][1] - Rtwn[2][1]*Rtwns[1][1] + Rtwn[1][2]*Rtwns[2][2] - Rtwn[2][2]*Rtwns[1][2] );
    ea[1] = -0.5*( Rtwn[2][0]*Rtwns[0][0] - Rtwn[0][0]*Rtwns[2][0] + Rtwn[2][1]*Rtwns[0][1] - Rtwn[0][1]*Rtwns[2][1] + Rtwn[2][2]*Rtwns[0][2] - Rtwn[0][2]*Rtwns[2][2] );
    ea[2] = -0.5*( Rtwn[0][0]*Rtwns[1][0] - Rtwn[1][0]*Rtwns[0][0] + Rtwn[0][1]*Rtwns[1][1] - Rtwn[1][1]*Rtwns[0][1] + Rtwn[0][2]*Rtwns[1][2] - Rtwn[1][2]*Rtwns[0][2] );
    vctFixedSizeVector<double,6> e( ex[0],ex[1],ex[2],ea[0],ea[1],ea[2] ); // 6-vector for position and orientation error 
    vctFixedSizeVector<double,6> ed = xd - xds;

    // Calculate control force in Cartesian space, world frame
    vctFixedSizeVector<double,6> FM = -1.0*Ki*xdd - Kp*e - Kd*ed;
    //vctFixedSizeVector<double,6> FM = -1.0*Kp*e - Kd*ed;
    
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


    // Add in inertia compensation (feedforward term) calculated in jointspace.
    // First translate the desired velocities into body frame, because we need the desired
    // acceleration in this frame (since the bias acceleration term is associated with the
    // body Jacobian).
    // Here we get the spatial vw vector as defined by MLS
/*    vctFixedSizeVector<double,3> vsb( 0.0 ), wsb( 0.0 );
    vsb[0] = xds[0] - xds[4]*xs[2] + xds[5]*xs[1];
    vsb[1] = xds[1] - xds[5]*xs[0] + xds[3]*xs[2];
    vsb[2] = xds[2] - xds[3]*xs[1] + xds[4]*xs[0];
    wsb[0] = xds[3];
    wsb[1] = xds[4];
    wsb[2] = xds[5];
    // Then we turn it into the body vw vector, again as defined by MLS
    // Need Rwns transpose
    vctMatrixRotation3<double> Rnws( Rtwns[0][0], Rtwns[1][0], Rtwns[2][0],
				     Rtwns[0][1], Rtwns[1][1], Rtwns[2][1],
				     Rtwns[0][2], Rtwns[1][2], Rtwns[2][2] );
    // also need xs-hat
    vctFixedSizeMatrix<double,3,3> xshat(   0.0, -xs[2],  xs[1],
    					  xs[2],    0.0, -xs[0],
					 -xs[1],  xs[0],    0.0	);
    // now do calculate body vw vector
    vsb = Rnws*vsb - Rnws*xshat*wsb;
    wsb = Rnws*wsb;
    // put it into a single vector
    xdsb[0] = vsb[0];
    xdsb[1] = vsb[1];
    xdsb[2] = vsb[2];
    xdsb[3] = wsb[0];
    xdsb[4] = wsb[1];
    xdsb[5] = wsb[2];
    // differentiate to get xdds
    xddsb = (xdsb - xdsbold)/dt;
    //need desired joint angles and velocities to calculate bias acceleration
    vctDynamicVector<double> qs(links.size(), 0.0);
    qs = q;
*/

//    InverseKinematics(qs,Rtwns);
//    vctDynamicVector<double> qds = (qs - qsold)/dt;
    // calculate bias acceleration
//    vctFixedSizeVector<double,6> Jdqd = BiasAcceleration(qs,qds);

    //
    // M is a 6x6, multiplies a 6-vector of translational and rotational accelerations
    // For inertia compensation, multiply by desired acceleration, not actual acceleration.
    //
    // First translate the desired velocities into body frame, because we need the desired
    // acceleration in this frame.
    // Here we get the spatial vw vector as defined by MLS
/*    vctFixedSizeVector<double,3> vsb( 0.0 ), wsb( 0.0 );
    vsb[0] = xds[0] - xds[4]*xs[2] + xds[5]*xs[1];
    vsb[1] = xds[1] - xds[5]*xs[0] + xds[3]*xs[2];
    vsb[2] = xds[2] - xds[3]*xs[1] + xds[4]*xs[0];
    wsb[0] = xds[3];
    wsb[1] = xds[4];
    wsb[2] = xds[5];
    // Then we turn it into the body vw vector, again as defined by MLS
    // Need Rwns transpose
    vctMatrixRotation3<double> Rnws( Rtwns[0][0], Rtwns[1][0], Rtwns[2][0],
				     Rtwns[0][1], Rtwns[1][1], Rtwns[2][1],
				     Rtwns[0][2], Rtwns[1][2], Rtwns[2][2] );
    // also need xs-hat
    vctFixedSizeMatrix<double,3,3> xshat(   0.0, -xs[2],  xs[1],
    					  xs[2],    0.0, -xs[0],
					 -xs[1],  xs[0],    0.0	);
    // now calculate body vw vector
    vsb = Rnws*vsb - Rnws*xshat*wsb;
    wsb = Rnws*wsb;
    // put it into a single vector
    xdsb[0] = vsb[0];
    xdsb[1] = vsb[1];
    xdsb[2] = vsb[2];
    xdsb[3] = wsb[0];
    xdsb[4] = wsb[1];
    xdsb[5] = wsb[2];
    // differentiate to get xdds
    xddsb = (xdsb - xdsbold)/dt;

    double M[6][6];
    OSinertia( M,q ); // Cartesian inertia in the body frame
    vctFixedSizeVector<double,6> IC( 0.0 ); // forces for inertia compensation
    for (int i=0;i<6;i++) {
      IC[i] = 0.0;
      for (int j=0;j<6;j++) {
        IC[i] += M[i][j]*xddsb[j];
      }
    }
    // Add in the resulting joint torques.
    // Since these forces are in the end effector frame, use the body Jacobian.
    // Note that the Jacobian is in column major, so indices are switched.
    for (unsigned int i=0;i<links.size();i++) {
      for (int j=0;j<6;j++) {
        tau[i] += 0.1*Jn[i][j]*IC[j];
      }
    }
  */

    // Add in the coriolis+gravity load and inertia compensation
    vctDynamicVector<double> ccg = CCG( q, qd );
    vctDynamicMatrix<double> M_JS = JSinertia( q );
    tau = tau + ccg + M_JS*qdd;
    //tau = tau + ccg;

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
        outputfile << "qd_raw(:," << count/steps << ") = [" << qd << "];" << std::endl;
        outputfile << "qd_filt(:," << count/steps << ") = [" << qd_filt << "];" << std::endl;
        outputfile << "x(:," << count/steps << ") = [" << x << "];" << std::endl;
        outputfile << "xs(:," << count/steps << ") = [" << xs << "];" << std::endl;
        outputfile << "Rtwns{" << count/steps << "} = reshape([" << Rtwns << "],4,4);" << std::endl;
        outputfile << "xds(:," << count/steps << ") = [" << xds << "];" << std::endl;
        outputfile << "vw(:," << count/steps << ") = [" << vw << "];" << std::endl;
        outputfile << "cutoff" << " = " << cutoff << ";" << std::endl;
        outputfile << "FM(:," << count/steps << ") = [" << FM << "];" << std::endl;
        outputfile << "tau(:," << count/steps << ") = [" << tau << "];" << std::endl;
        outputfile << "qdd_raw(:," << count/steps << ") = [" << qdd_unfilt << "];" << std::endl;
        outputfile << "qdd_filt(:," << count/steps << ") = [" << qdd << "];" << std::endl;
        outputfile << "xd_raw(:," << count/steps << ") = [" << xd_unfilt << "];" << std::endl;
        outputfile << "xd_used(:," << count/steps << ") = [" << xd << "];" << std::endl;
        outputfile << "xd_filt(:," << count/steps << ") = [" << xd_filt << "];" << std::endl;
        outputfile << "xdd_raw(:," << count/steps << ") = [" << xdd_unfilt << "];" << std::endl;
        outputfile << "xdd_filt(:," << count/steps << ") = [" << xdd << "];" << std::endl;
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
  qdold = qd;
  qd_filt_old = qd_filt;
  qddold = qdd;
  xdold = xd;
  xddold = xdd;
  xd_filt_old = xd_filt;
//  qsold = qs;
  told = t;
  xold = x;
  xdsbold = xdsb;
//tau[1] += 0.02;
  //	    << "tau: " << tau << std::endl;
  output->SetForceTorque( tau );

  
}
