#include <cisstDevices/can/devRTSocketCAN.h>

#include <cisstCommon/cmnConstants.h>

#include "wam_impedance_experiment/devices/devKeyboardAAB.h"

// To use a trajectory
#include <cisstDevices/robotcomponents/trajectories/devSetPoints.h>
#include <cisstDevices/robotcomponents/trajectories/devLinearSE3.h>
#include "wam_impedance_experiment/trajectories/devForceInput.h"

#include "wam_impedance_experiment/robotcomponents/devWAM_AAB.h"

// To use force sensor
#include "wam_impedance_experiment/devices/devDAS6014.h"
#include <cisstMultiTask/mtsTaskPeriodic.h>
#include <cisstMultiTask/mtsInterfaceRequired.h>

// To use a control loop
#include "wam_impedance_experiment/controllers/devImpedance.h"
#include "wam_impedance_experiment/controllers/devImpedanceTunable.h"

// To run the show
#include <cisstMultiTask/mtsTaskManager.h>
#include <cisstOSAbstraction/osaSleep.h>

using namespace std;

int main(int argc, char** argv){

  mtsTaskManager* taskManager = mtsTaskManager::GetInstance();

  devKeyboard keyboard;
  keyboard.SetQuitKey('q');
  keyboard.AddKeyWriteFunction( 'n', "NextSetPoint", devSetPoints::NextSetPoint, true);
  keyboard.AddKeyWriteFunction( 'G', "ctrlenable", devController::Enable, true );
  keyboard.AddKeyWriteFunction( 't', "DisableTrajectory", devLinearSE3::Enable, false);
  keyboard.AddKeyWriteFunction( 't', "EnableFin", devForceInput::Enable, true);
  keyboard.AddKeyWriteFunction( 't', "DisableConstantImpedance", devController::Enable, false);
  keyboard.AddKeyWriteFunction( 't', "EnableVariableImpedance", devController::Enable, true);
  taskManager->AddComponent( &keyboard );

  devRTSocketCAN can( "rtcan0", devCAN::RATE_1000 ); // rtcan0 = left WAM, rtcan1 = right WAM

  vctDynamicVector<double> qinit(7, 0.0);              // Initial joint values
  qinit[1] = -cmnPI_2;
  qinit[3] =  cmnPI;
  qinit[4] = -cmnPI_2;
  qinit[5] = -cmnPI_2;

  devWAM_AAB wam( "WAM", 0.002, OSA_CPU1, &can, qinit );
  wam.Configure();
  taskManager->AddComponent( &wam );

  std::string path("libs/etc/cisstRobot/WAM/");

  vctMatrixRotation3<double> Rw0(  0.0, 0.0, -1.0,
                                   0.0, 1.0, 0.0,
                                   1.0, 0.0, 0.0 );
  vctFixedSizeVector<double,3> tw0(0.0, 0.0, 1.0);
  vctFrame4x4<double> Rtw0(Rw0,tw0);                   // base transformation

  // get initial end effector position and orientation in world frame
  robManipulator WAMtmp( "libs/etc/cisstRobot/WAM/wam7.rob", Rtw0 );
  vctFrame4x4<double> Rtwninit = WAMtmp.ForwardKinematics( qinit );

  // Create a set point generator
  std::vector< vctFrame4x4<double> > Rtwn;

  vctFixedSizeVector<double,3> t0 = Rtwninit.Translation();
  vctFixedSizeVector<double,3> tC = t0;
  tC[0] = -0.85;
  tC[2] = 0.5;
  vctFixedSizeVector<double,3> tL = tC;
  tL[1] = -0.4; // left limit of usable workspace
  vctFixedSizeVector<double,3> tR = tC;
  tR[1] = 0.6; // right limit of usable workspace
  vctFixedSizeVector<double,3> t1 = tR;
  t1[0] = -0.6;

  vctMatrixRotation3<double> RwU(  1,  0,  0,
                                   0,  1,  0,
                                   0,  0,  1 ); // point up (this is the initial orientation)
  vctMatrixRotation3<double> RwD( -1,  0,  0,
                                   0,  1,  0,
                                   0,  0, -1 ); // point down
  vctMatrixRotation3<double> RwL( -1,  0,  0,
                                   0,  0, -1,
                                   0, -1,  0 ); // point to WAM left
  vctMatrixRotation3<double> RwR( -1,  0,  0,
                                   0,  0,  1,
                                   0,  1,  0 ); // point to WAM right
  vctMatrixRotation3<double> RwO(  0,  0, -1,
                                   0,  1,  0,
                                   1,  0,  0 ); // point out

  vctFrame4x4<double> RtwnC( RwR, tC );
  vctFrame4x4<double> RtwnL( RwR, tL );
  vctFrame4x4<double> RtwnR( RwR, tR );
  vctFrame4x4<double> Rtwn1( RwR, t1 );

  Rtwn.push_back( Rtwninit );
  Rtwn.push_back( RtwnC );
  Rtwn.push_back( RtwnL );
/*
  Rtwn.push_back( Rtwn1 );
  Rtwn.push_back( RtwnR );
  Rtwn.push_back( RtwnL );
  Rtwn.push_back( RtwnR );
  Rtwn.push_back( RtwnL );
  Rtwn.push_back( RtwnR );
  Rtwn.push_back( RtwnL );
  Rtwn.push_back( RtwnR );
  Rtwn.push_back( RtwnL );
  Rtwn.push_back( RtwnR );
*/
  Rtwn.push_back( RtwnC );
  Rtwn.push_back( Rtwninit );
  devSetPoints setpoints( "setpoints", Rtwn );
  taskManager->AddComponent( &setpoints );

  // Create a trajectory

  devLinearSE3 trajectory( "trajectory",          // The name of the trajectory
      0.001,                  //The trajectory runs at 100Hz
      devTrajectory::ENABLED,// initial state
      OSA_CPU2,
      devTrajectory::QUEUE,
      devTrajectory::POSITION,
      Rtwninit,
      0.3,              // Max linear velocity
      0.3 );            // Max angular velocity
  taskManager->AddComponent( &trajectory );

  devForceInput ForceInput( "ForceInput",
      0.001,
      devTrajectory::DISABLED,
      OSA_CPU2,
      "libs/etc/cisstRobot/WAM/wam7.rob",
      Rtw0, 
      devTrajectory::POSITION |
      devTrajectory::VELOCITY,
      RtwnC );
  taskManager->AddComponent( &ForceInput );

  // Force sensor
  vctDynamicVector<unsigned int> channels( 2, 0, 1 ); // two channels: 0,1
  devDAS6014 das6014( "/dev/comedi0", 0, channels );
  taskManager->AddComponent( &das6014 );

  // Create the controller
  vctFixedSizeMatrix<double,6,6> Kp(0.0), Kd(0.0);
  Kp[0][0] = 500.0;	Kd[0][0] = 20.0;
  Kp[1][1] = 500.0;	Kd[1][1] = 20.0;
  Kp[2][2] = 500.0;	Kd[2][2] = 20.0;
  Kp[3][3] = 4.0;	Kd[3][3] = 0.1;
  Kp[4][4] = 1.0;	Kd[4][4] = 0.1;
  Kp[5][5] = 4.0;	Kd[5][5] = 0.1;

  devImpedance controllerC( "controllerC",
			          0.002,
                                  devController::DISABLED,// initial state
			          OSA_CPU3,
		 	          "libs/etc/cisstRobot/WAM/wam7.rob",
			          Rtw0, 
			          Kp, 
			          Kd,
				  "datafile.txt" );
  taskManager->AddComponent( &controllerC );

  devImpedanceTunable controllerV( "controllerV",
			          0.002,
                                  devController::DISABLED,// initial state
			          OSA_CPU3,
		 	          "libs/etc/cisstRobot/WAM/wam7.rob",
			          Rtw0, 
			          Kp, 
			          Kd,
				  "datafile2.txt" );
  taskManager->AddComponent( &controllerV );


  taskManager->Connect( keyboard.GetName(),   "NextSetPoint",
			setpoints.GetName(),  devSetPoints::Control );

  taskManager->Connect( keyboard.GetName(),     "DisableConstantImpedance",
                        controllerC.GetName(),  devTrajectory::Control );

  taskManager->Connect( keyboard.GetName(),     "EnableVariableImpedance",
                        controllerV.GetName(),  devTrajectory::Control );

  taskManager->Connect( keyboard.GetName(),   "DisableTrajectory",
                        trajectory.GetName(), devTrajectory::Control );

  taskManager->Connect( keyboard.GetName(),   "EnableFin",
                        ForceInput.GetName(), devTrajectory::Control );

  taskManager->Connect( keyboard.GetName(),   "ctrlenable",
                        controllerC.GetName(), devController::Control );

  taskManager->Connect( trajectory.GetName(), devTrajectory::Input,
			setpoints.GetName(),  devSetPoints::Output );

  taskManager->Connect( trajectory.GetName(), devTrajectory::Output,
			controllerC.GetName(), devController::Input );

  taskManager->Connect( das6014.GetName(),    devDAS6014::Output,
                        ForceInput.GetName(), "Input" );

  taskManager->Connect( das6014.GetName(),    devDAS6014::Output,
                        controllerV.GetName(), "Input2" );

  taskManager->Connect( ForceInput.GetName(), devTrajectory::Output,
                        controllerV.GetName(), devController::Input );

  taskManager->Connect( controllerC.GetName(), devController::Output,
			wam.GetName(),        devManipulator::Input );

  taskManager->Connect( controllerC.GetName(), devController::Feedback,
			wam.GetName(),        devManipulator::Output );

  taskManager->Connect( controllerV.GetName(), devController::Output,
			wam.GetName(),        devManipulator::Input );

  taskManager->Connect( controllerV.GetName(), devController::Feedback,
			wam.GetName(),        devManipulator::Output );

  // connect WAM output to trajectory feedback inputs, if needed
  taskManager->Connect( ForceInput.GetName(),
                        devForceInput::WamStateInput,
                        wam.GetName(),
                        devManipulator::Output );

  taskManager->CreateAll();


  keyboard.Start();
  setpoints.Start();
  wam.Start();
  osaSleep(0.1);
  trajectory.Start();
  osaSleep(0.1);
  das6014.Start();
  osaSleep(0.1);
  ForceInput.Start();
  osaSleep(0.1);
  controllerC.Start();

  controllerV.Start();

//  taskManager->StartAll();
  pause();

  return 0;
}
