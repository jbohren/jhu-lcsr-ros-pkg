


#ifndef _devVelEqn_h
#define _devVelEqn_h

#include "wam_impedance_experiment/robotcomponents/robManipulator_AAB.h"
#include <cisstDevices/robotcomponents/trajectories/devTrajectory.h>
#include <cisstDevices/devExport.h>


class CISST_EXPORT devVelEqn : 
  public devTrajectory,
  public robManipulator_AAB {

 private:

  SE3IO*  input;
  SE3IO* output;
  RnIO* wam_state;

  //const std::string& robfile;
  //const vctFrame4x4<double>& Rtw0;
  vctFrame4x4<double> Rtinit, Rtold;
  vctFrame4x4<double> Rtp;
  vctFrame4x4<double> Rtn;
  //robManipulator_AAB wam;

  bool firstTime; // this is a hack - I need a way to capture the start time of the trajectory
  bool outOfRange; // also need a way to keep track of when the start position is out of range, and when it comes back in range
  double t_start;
  double told, t1old;
  double phase;
  int count;
  double F, Fold;

  vctFixedSizeVector<double,6> vw, vdwd;
  bool IsInputNew();

  robFunction* Queue( double t, robFunction* function );
  robFunction* Track( double t, robFunction* function );
  
 protected:

  //! Evaluate a function
  void Evaluate( double t, robFunction* function );

 public:

  devVelEqn( const std::string& name, 
		double period, 
		devTrajectory::State state,
		osaCPUMask cpumask,
		const std::string& robfile,
		const vctFrame4x4<double>& Rtw0, 
		devTrajectory::Variables variables,
    const vctFrame4x4<double>& Rtinit ) ;

  //void Startup();

  mtsFunctionQualifiedRead ReadVolt;
  mtsFunctionQualifiedRead GetZero;
  
  //! Name of the trajectory interface to the WAM
  static const std::string WamStateInput;

};

#endif
