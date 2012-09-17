
#ifndef _devMultipleControllers_h
#define _devMultipleControllers_h

#include "wam_impedance_experiment/robotcomponents/robManipulator_AAB.h"
#include <cisstDevices/robotcomponents/controllers/devController.h>
#include <cisstDevices/devExport.h>
#include <cisstMultiTask/mtsMatrix.h>
#include "wam_impedance_experiment/ControllerType.h"


class ControllerData : public mtsGenericObject {
  CMN_DECLARE_SERVICES(CMN_DYNAMIC_CREATION, CMN_LOG_ALLOW_DEFAULT);

public:
  ControllerData() :
    t(0),
    dt(0),
    Kp_null(0.0),
    Kd_null(0.0),
    friction_gain(0.0),
    impulse(0.0),
    forceExternal(0.0),
    controllerNumber(0),
    theta(0.0),
    m_y(0.0),
    tracking_goal(0.0)
  {
    q.resize(7);
    qs.resize(7);
    qd.resize(7);
    qsd.resize(7);
    setpoint_null.resize(7);
    tau_gc.resize(7);
    tau_ct.resize(7);
    tau_imp.resize(7);
    tau_imp_null.resize(7);
    tau_imp_null_fric.resize(7);
    tau_external.resize(7);
    tau.resize(7);
    Kp_Rn.resize(7);
    Kd_Rn.resize(7);
    Kp_SE3.resize(6);
    Kd_SE3.resize(6);
    Rtwn.resize(4,4);
    Rtwns.resize(4,4);
    xds.resize(6);
    erroraxis.resize(3);
    M.resize(3,3);
    M_OS.resize(6,6);
    M_JS.resize(7,7);
  }

  double t;
  double dt;
  mtsVector<double> q;
  mtsVector<double> qs;
  mtsVector<double> qd;
  mtsVector<double> qsd;
  mtsVector<double> setpoint_null;
  mtsVector<double> tau_gc;
  mtsVector<double> tau_ct;
  mtsVector<double> tau_imp;
  mtsVector<double> tau_imp_null;
  mtsVector<double> tau_imp_null_fric;
  mtsVector<double> tau_external;
  mtsVector<double> tau;
  mtsVector<double> Kp_Rn;
  mtsVector<double> Kd_Rn;
  mtsVector<double> Kp_SE3;
  mtsVector<double> Kd_SE3;
  double Kp_null;
  double Kd_null;
  double friction_gain;
  double impulse;
  double forceExternal;
  mtsMatrix<double> Rtwn;
  mtsMatrix<double> Rtwns;
  mtsVector<double> xds;
  int controllerNumber;
  mtsVector<double> erroraxis;
  double theta;
  double m_y;
  mtsMatrix<double> M;
  mtsMatrix<double> M_OS;
  mtsMatrix<double> M_JS;
  double tracking_goal;

};

CMN_DECLARE_SERVICES_INSTANTIATION(ControllerData);

class ControlInfo : public mtsGenericObject {
  CMN_DECLARE_SERVICES(CMN_DYNAMIC_CREATION, CMN_LOG_ALLOW_DEFAULT);

public: 
  ControlInfo() : 
    controllerNumber(0),
    stiffness(0),
    damping(0)
  {
    setpoint_null.resize(7);
  }



  int controllerNumber;
  double stiffness;
  double damping;
  mtsVector<double> setpoint_null;

};

//typedef mtsGenericObjectProxy<ControlInfo> ControlInfoProxy;
CMN_DECLARE_SERVICES_INSTANTIATION(ControlInfo);

class CISST_EXPORT devMultipleControllers : 

  public devController,
  public robManipulator_AAB {

 private:

  const std::string name;

  SE3IO* inputSE3;
  RnIO* inputRn;
  RnIO* output;
  RnIO* feedback;

  controllerType controller;
  vctDynamicMatrix<double> Kp_Rn;
  vctDynamicMatrix<double> Kd_Rn;
  vctFixedSizeMatrix<double,6,6> Kp_SE3;
  vctFixedSizeMatrix<double,6,6> Kd_SE3;
  double Kp_null;
  double Kd_null;
  double friction_gain;
  double impulse;
  double impulseTime;
  int nImpulseSteps;
  int countImpulseSteps;
  double dt_nominal;
  vctDynamicVector<double> forceExternal;

  vctDynamicVector<double> qold;
  vctDynamicVector<double> setpoint_null;
  vctDynamicVector<double> qdold;
  vctDynamicVector<double> qdfiltold;
  vctDynamicVector<double> eold;
  vctFixedSizeVector<double,3> erroraxis;
  double theta;
  vctFixedSizeVector<double,3> xold;
  vctFixedSizeVector<double,6> xdold;
  vctDynamicVector<double> tauold;
  vctDynamicVector<double> tau_gc;
  vctDynamicVector<double> tau_ct;
  vctDynamicVector<double> tau_imp;
  vctDynamicVector<double> tau_imp_null;
  vctDynamicVector<double> tau_imp_null_fric;
  vctDynamicVector<double> tau_external;
  vctFixedSizeVector<double,7> saturation_limits;

  // need three sets of timestamps: 
  //  one for input coming from the wam feedback,
  //  one for input coming from the trajectory,
  //  and one for the control loop itself.
  double told;
  double told_fb;

  // for controller transitions
  double transitionTime;
  double t_transition;
  bool newController;
  vctDynamicVector<double> tau_transition;

  double tracking_goal;

 protected:

  void Evaluate();

 public:

  devMultipleControllers( const std::string& name, 
			  double period,
			  devController::State state,
			  osaCPUMask mask,
			  const std::string& robfile,
			  const vctFrame4x4<double>& Rtwb,
        const vctDynamicMatrix<double>& Kp_Rn,
        const vctDynamicMatrix<double>& Kd_Rn,
        vctFixedSizeMatrix<double,6,6>& Kp_SE3,
        vctFixedSizeMatrix<double,6,6>& Kd_SE3 );

  void SetControllerInfo(const ControlInfo& data);
  void SetGains(const mtsVector<double>& data);
  void AddImpulse(const double& data);
  void GetTrackingGoal(const double& data);
  //void ResetTrajectoryDone(void);
  mtsFunctionWrite PublishControllerData;
  //mtsFunctionVoid ResetTrajectory;

};

#endif
