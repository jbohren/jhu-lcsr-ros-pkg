
#ifndef _devImpedanceTunable_h
#define _devImpedanceTunable_h

#include <cisstVector/vctFixedSizeMatrix.h>
#include "wam_impedance_experiment/robotcomponents/robManipulator_AAB.h"

#include <cisstDevices/robotcomponents/controllers/devController.h>
#include <cisstDevices/devExport.h>

class CISST_EXPORT devImpedanceTunable : 
  public devController,
  public robManipulator_AAB {

 private:
  
  SE3IO* input;
  RnIO* output;
  RnIO* feedback;

  vctFixedSizeMatrix<double,6,6> Kp;
  vctFixedSizeMatrix<double,6,6> Kd;
  const std::string datafile;

  vctDynamicVector<double> qold;
  vctDynamicVector<double> qsold;
  vctFixedSizeVector<double,6> eold;
  vctFixedSizeVector<double,3> xold;
  vctFixedSizeVector<double,6> xdold;
  vctFixedSizeVector<double,6> xdsold;
  vctFixedSizeVector<double,6> xdsbold;

  double told;

  bool EnableOutput;
  int count;
  bool FirstWrite;

  protected:

  void Evaluate();

 public:

  devImpedanceTunable( const std::string& taskname, 
		       double period,
		       devController::State state,
		       osaCPUMask mask,
	 	       const std::string& robfile,
		       const vctFrame4x4<double>& Rtw0, 
		       vctFixedSizeMatrix<double,6,6>& Kp,
		       vctFixedSizeMatrix<double,6,6>& Kd,
		       const std::string datafile );

  mtsFunctionQualifiedRead ReadVolt;
  mtsFunctionQualifiedRead GetZero;

};

#endif
