
#ifndef _devImpedanceIC_h
#define _devImpedanceIC_h

#include <cisstVector/vctFixedSizeMatrix.h>
#include "wam_impedance_experiment/robotcomponents/robManipulator_AAB.h"

#include <cisstDevices/robotcomponents/controllers/devController.h>
#include <cisstDevices/devExport.h>

class CISST_EXPORT devImpedanceIC : 
  public devController,
  public robManipulator_AAB {

 private:
  
  SE3IO* input;
  RnIO* output;
  RnIO* feedback;

  vctFixedSizeMatrix<double,6,6> Ki;
  vctFixedSizeMatrix<double,6,6> Kp;
  vctFixedSizeMatrix<double,6,6> Kd;
  const std::string datafile;

  vctDynamicVector<double> qold;
  vctDynamicVector<double> qdold;
  vctDynamicVector<double> qddold;
  vctDynamicVector<double> qsold;
  vctDynamicVector<double> qd_filt_old;
  vctFixedSizeVector<double,6> eold;
  vctFixedSizeVector<double,3> xold;
  vctFixedSizeVector<double,6> xdold;
  vctFixedSizeVector<double,6> xddold;
  vctFixedSizeVector<double,6> xd_filt_old;
  vctFixedSizeVector<double,6> xdsold;
  vctFixedSizeVector<double,6> xdsbold;

  double told;

  // temp for getting inertia values
  bool EnableOutput;
  int count;
  bool FirstWrite;

  protected:

  void Evaluate();

 public:

  devImpedanceIC( const std::string& taskname, 
		double period,
		devController::State state,
		osaCPUMask mask,
		const std::string& robfile,
		const vctFrame4x4<double>& Rtw0, 
		const vctFixedSizeMatrix<double,6,6>& Ki,
		const vctFixedSizeMatrix<double,6,6>& Kp,
		const vctFixedSizeMatrix<double,6,6>& Kd,
		const std::string datafile );

};

#endif
