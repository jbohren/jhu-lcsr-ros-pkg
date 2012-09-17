


#ifndef _devReturnToZero_h
#define _devReturnToZero_h

#include "wam_impedance_experiment/robotcomponents/robManipulator_AAB.h"
#include <cisstDevices/robotcomponents/trajectories/devTrajectory.h>
#include <cisstDevices/devExport.h>


class CISST_EXPORT devReturnToZero : 
  public devTrajectory,
  public robManipulator_AAB {

 private:

  SE3IO*  input;
  SE3IO* output;
  RnIO* wam_state;

  vctFrame4x4<double> Rthome, Rtold;
  vctFrame4x4<double> Rtp;
  vctFrame4x4<double> Rtn;

  bool firstTime; // this is a hack - I need a way to capture the start time of the trajectory
  double told, t1old;
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

  devReturnToZero( const std::string& name, 
		double period, 
		devTrajectory::State state,
		osaCPUMask cpumask,
		const std::string& robfile,
		const vctFrame4x4<double>& Rtw0, 
		devTrajectory::Variables variables,
    const vctFrame4x4<double>& Rthome ) ;

  //void Startup();

  mtsFunctionQualifiedRead ReadVolt;
  mtsFunctionQualifiedRead GetZero;

  //! Name of the trajectory interface to the WAM
  static const std::string WamStateInput;
};

#endif
