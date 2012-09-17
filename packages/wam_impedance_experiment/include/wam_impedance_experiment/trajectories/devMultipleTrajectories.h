
#ifndef _devMultipleTrajectories_h
#define _devMultipleTrajectories_h

#include "wam_impedance_experiment/robotcomponents/robManipulator_AAB.h"
#include <cisstDevices/robotcomponents/trajectories/devTrajectory.h>
#include <cisstDevices/devExport.h>
#include "wam_impedance_experiment/TrajectoryType.h"


class TrajInfo : public mtsGenericObject {
  CMN_DECLARE_SERVICES(CMN_DYNAMIC_CREATION, CMN_LOG_ALLOW_DEFAULT);

public:
  TrajInfo() :
    t(0),
    trajectoryNumber(0),
    direction(0),
    setpointRn(0.0),
    maxCommandSpeed(false)
  {
  }

  double t;
  int trajectoryNumber;
  int direction;
  vctDynamicVector<double> setpointRn;
  vctFrame4x4<double> setpointSE3;
  vctFixedSizeVector<double,6> velocitySE3;
  bool maxCommandSpeed;

};

CMN_DECLARE_SERVICES_INSTANTIATION(TrajInfo);

class CISST_EXPORT devMultipleTrajectories : 
  public devTrajectory,
  public robManipulator_AAB {

 private:

  SE3IO* outputSE3;
  RnIO* outputRn;
  RnIO* wam_feedback;

  trajectoryType trajectory;
  int direction;
  vctDynamicVector<double> setpointRn;
  vctFrame4x4<double> setpointSE3;
  vctDynamicVector<double> savedPositionRn;
  vctFrame4x4<double> savedPositionSE3;
  bool newTrajectory;
  bool newSetpoint;
  //bool newController;
  //bool resettingTrajectory;
  double userCommandVelocity;
  bool maxCommandSpeed;

  vctDynamicVector<double> qhome;
  vctDynamicVector<double> qold;
  const bool useActualPosition; // this is a hack. tells the trajectory where to start from when switching between joint trajectory and SE3 trajectory - measured wam position or position from inverse kinematics
  const double limitL; // left limit of workspace for user input trajectory
  const double limitR; // right limit of workspace for user input trajectory
  const double thetadmax;
  const double vmax;
  const double wmax;
  vctFrame4x4<double> Rtold;

  double told;

  // for sinusoid trajectory
  double t_start;

  double admittance;
  double deadband;

  vctFixedSizeVector<double,6> vw, vdwd;
  vctDynamicVector<double> qd, qdd;
  bool IsInputNew();

  robFunction* Queue( double t, robFunction* function );
  robFunction* Track( double t, robFunction* function );
  
 protected:

  //! Evaluate a function
  void Evaluate( double t, robFunction* function );

 public:

  devMultipleTrajectories( const std::string& name, 
		double period, 
		devTrajectory::State state,
		osaCPUMask cpumask,
		const std::string& robfile,
		const vctFrame4x4<double>& Rtw0, 
		devTrajectory::Variables variables,
    const vctDynamicVector<double>& qhome,
    const bool useActualPosition,
    const double limitL = -0.45,
    const double limitR = 0.45,
    const double thetadmax = 0.4,
    const double vmax = 0.3,
    const double wmax = 0.5);

  //void ResetTrajectory(void);
  void SetTrajectoryInfo(const TrajInfo& value);
  void HandleUserInput(const double& value);
  mtsFunctionWrite PublishTrajectory;
  //mtsFunctionVoid ResetTrajectoryDone;

  //void Startup();

};

#endif
