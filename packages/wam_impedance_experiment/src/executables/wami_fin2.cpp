#include <cisstDevices/can/devRTSocketCAN.h>

#include <cisstCommon/cmnConstants.h>

#include "wam_impedance_experiment/devices/devKeyboardAAB.h"

// To use a trajectory
#include <cisstDevices/robotcomponents/trajectories/devSetPoints.h>
#include <cisstDevices/robotcomponents/trajectories/devLinearSE3.h>
#include "wam_impedance_experiment/trajectories/devForceInput.h"
#include "wam_impedance_experiment/trajectories/devVelEqn.h"

#include "wam_impedance_experiment/robotcomponents/devWAM_AAB.h"

// To use force sensor
#include "wam_impedance_experiment/devices/devDAS6014.h"
#include <cisstMultiTask/mtsTaskPeriodic.h>
#include <cisstMultiTask/mtsInterfaceRequired.h>

// To use a control loop
#include "wam_impedance_experiment/controllers/devImpedance.h"
#include "wam_impedance_experiment/controllers/devImpedanceTunable.h"
//#include <cisstDevices/robotcomponents/controllers/devImpedanceTunable2.h>

// To run the show
#include <cisstMultiTask/mtsTaskManager.h>
#include <cisstOSAbstraction/osaSleep.h>

using namespace std;

int main(int argc, char** argv){

  mtsTaskManager* taskManager = mtsTaskManager::GetInstance();

  devKeyboard keyboard;
  keyboard.SetQuitKey('q');
  keyboard.AddKeyWriteFunction( 'n', "NextSetPointL", devSetPoints::NextSetPoint, true);
  keyboard.AddKeyWriteFunction( 'm', "NextSetPointR", devSetPoints::NextSetPoint, true);
  keyboard.AddKeyWriteFunction( 'G', "ctrlenableL", devController::Enable, true );
  keyboard.AddKeyWriteFunction( 'G', "ctrlenableR", devController::Enable, true );
  keyboard.AddKeyWriteFunction( 't', "DisableTrajectoryL", devLinearSE3::Enable, false);
  keyboard.AddKeyWriteFunction( 't', "DisableTrajectoryR", devLinearSE3::Enable, false);
  keyboard.AddKeyWriteFunction( 't', "EnableFinL", devForceInput::Enable, true);
  keyboard.AddKeyWriteFunction( 't', "EnableVelEqnR", devVelEqn::Enable, true);
  keyboard.AddKeyWriteFunction( 't', "DisableConstantImpedanceL", devController::Enable, false);
  keyboard.AddKeyWriteFunction( 't', "DisableConstantImpedanceR", devController::Enable, false);
  keyboard.AddKeyWriteFunction( 't', "EnableVariableImpedanceL", devController::Enable, true);
  keyboard.AddKeyWriteFunction( 't', "EnableVariableImpedanceR", devController::Enable, true);

  taskManager->AddComponent( &keyboard );

  devRTSocketCAN canL( "rtcan0", devCAN::RATE_1000 );
  devRTSocketCAN canR( "rtcan1", devCAN::RATE_1000 );

  vctDynamicVector<double> qinitL(7, 0.0);              // Initial joint values, left arm
  qinitL[1] = -cmnPI_2;
  qinitL[3] =  cmnPI;
  qinitL[4] = -cmnPI_2;
  qinitL[5] = -cmnPI_2;

  vctDynamicVector<double> qinitR(7, 0.0);              // Initial joint values, right arm
  qinitR[1] = -cmnPI_2;
  qinitR[3] =  cmnPI;
  qinitR[4] = -cmnPI_2;
  qinitR[5] =  cmnPI_2;

  devWAM_AAB wamL( "WAML", 0.002, OSA_CPU1, &canL, qinitL );
  wamL.Configure();
  taskManager->AddComponent( &wamL );

  devWAM_AAB wamR( "WAMR", 0.002, OSA_CPU1, &canR, qinitR );
  wamR.Configure();
  taskManager->AddComponent( &wamR );

  std::string path("libs/etc/cisstRobot/WAM/");

  vctMatrixRotation3<double> Rw0(  0.0, 0.0, -1.0,
                                   0.0, 1.0, 0.0,
                                   1.0, 0.0, 0.0 );
  vctFixedSizeVector<double,3> tw0(0.0, 0.0, 1.0);
  vctFrame4x4<double> Rtw0(Rw0,tw0);                   // base transformation

  // get initial end effector position and orientation in world frame
  robManipulator WAMtmp( "libs/etc/cisstRobot/WAM/wam7.rob", Rtw0 );
  vctFrame4x4<double> RtwninitL = WAMtmp.ForwardKinematics( qinitL );
  vctFrame4x4<double> RtwninitR = WAMtmp.ForwardKinematics( qinitR );

  // Create a set point generator
  std::vector< vctFrame4x4<double> > RtwnL;
  std::vector< vctFrame4x4<double> > RtwnR;

  vctFixedSizeVector<double,3> t0L = RtwninitL.Translation();
  vctFixedSizeVector<double,3> tCL = t0L;
  tCL[0] = -0.85;
  tCL[1] = 0;
  tCL[2] = 0.5;
  vctFixedSizeVector<double,3> tLL = tCL;
  tLL[1] = -0.4; // left limit of usable workspace (left arm)
  vctFixedSizeVector<double,3> tRL = tCL;
  tRL[1] = 0.6; // right limit of usable workspace (left arm)

  vctFixedSizeVector<double,3> t0R = RtwninitR.Translation();
  vctFixedSizeVector<double,3> tCR = t0R;
  tCR[0] = -0.85;
  tCR[1] = 0;
  tCR[2] = 0.5;
  vctFixedSizeVector<double,3> tLR = tCR;
  tLR[1] = -0.6; // left limit of usable workspace (right arm)
  vctFixedSizeVector<double,3> tRR = tCR;
  tRR[1] = 0.4; // right limit of usable workspace (right arm)

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

  // center, left, and right positions for left arm
  vctFrame4x4<double> RtwnCL( RwR, tCL );
  vctFrame4x4<double> RtwnLL( RwR, tLL );
  vctFrame4x4<double> RtwnRL( RwR, tRL );

  // center, left, and right positions for right arm
  vctFrame4x4<double> RtwnCR( RwL, tCR );
  vctFrame4x4<double> RtwnLR( RwL, tLR );
  vctFrame4x4<double> RtwnRR( RwL, tRR );

  RtwnL.push_back( RtwninitL );
  RtwnL.push_back( RtwnCL );
  RtwnL.push_back( RtwnLL );
/*
  RtwnL.push_back( RtwnRL );
  RtwnL.push_back( RtwnLL );
  RtwnL.push_back( RtwnRL );
  RtwnL.push_back( RtwnLL );
  RtwnL.push_back( RtwnRL );
  RtwnL.push_back( RtwnLL );
  RtwnL.push_back( RtwnRL );
  RtwnL.push_back( RtwnLL );
  RtwnL.push_back( RtwnRL );
*/
  RtwnL.push_back( RtwnCL );
  RtwnL.push_back( RtwninitL );
  devSetPoints setpointsL( "setpointsL", RtwnL );
  taskManager->AddComponent( &setpointsL );

  RtwnR.push_back( RtwninitR );
  RtwnR.push_back( RtwnCR );
  RtwnR.push_back( RtwnRR );
/*
  RtwnR.push_back( RtwnRR );
  RtwnR.push_back( RtwnLR );
  RtwnR.push_back( RtwnRR );
  RtwnR.push_back( RtwnLR );
  RtwnR.push_back( RtwnRR );
  RtwnR.push_back( RtwnLR );
  RtwnR.push_back( RtwnRR );
  RtwnR.push_back( RtwnLR );
  RtwnR.push_back( RtwnRR );
*/
  RtwnR.push_back( RtwnCR );
  RtwnR.push_back( RtwninitR );
  devSetPoints setpointsR( "setpointsR", RtwnR );
  taskManager->AddComponent( &setpointsR );

  // Create a trajectory

  devLinearSE3 trajectoryL( "trajectoryL",          // The name of the trajectory
      0.002,
      devTrajectory::ENABLED,// initial state
      OSA_CPU2,
      devTrajectory::QUEUE,
      devTrajectory::POSITION,
      RtwninitL,
      0.3,              // Max linear velocity
      0.3 );            // Max angular velocity
  taskManager->AddComponent( &trajectoryL );

  devLinearSE3 trajectoryR( "trajectoryR",          // The name of the trajectory
      0.002,
      devTrajectory::ENABLED,// initial state
      OSA_CPU2,
      devTrajectory::QUEUE,
      devTrajectory::POSITION,
      RtwninitR,
      0.3,              // Max linear velocity
      0.3 );            // Max angular velocity
  taskManager->AddComponent( &trajectoryR );

  devForceInput ForceInputL( "ForceInputL",
      0.002,
      devTrajectory::DISABLED,
      OSA_CPU2,
      "libs/etc/cisstRobot/WAM/wam7.rob",
      Rtw0, 
      devTrajectory::POSITION |
      devTrajectory::VELOCITY,
      RtwnCL );
  taskManager->AddComponent( &ForceInputL );

  devVelEqn VelEqnR( "VelEqnR",
      0.002,
      devTrajectory::DISABLED,
      OSA_CPU2,
      "libs/etc/cisstRobot/WAM/wam7.rob",
      Rtw0, 
      devTrajectory::POSITION |
      devTrajectory::VELOCITY,
      RtwnRR );
  taskManager->AddComponent( &VelEqnR );

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

  devImpedance controllerCR( "controllerCR",	// constant impedance controller for initial placement
      0.002,
      devController::DISABLED,// initial state
      OSA_CPU4,
      "libs/etc/cisstRobot/WAM/wam7.rob",
      Rtw0, 
      Kp, 
      Kd,
      "datafileR1.txt" );
  taskManager->AddComponent( &controllerCR );

  devImpedance controllerCL( "controllerCL",	// constant impedance controller for initial placement
			          0.002,
                                  devController::DISABLED,// initial state
			          OSA_CPU3,
		 	          "libs/etc/cisstRobot/WAM/wam7.rob",
			          Rtw0, 
			          Kp, 
			          Kd,
				  "datafileL1.txt" );
  taskManager->AddComponent( &controllerCL );

  devImpedanceTunable controllerVL( "controllerVL",	// variable impedance controller for user input
			          0.002,
                                  devController::DISABLED,// initial state
			          OSA_CPU3,
		 	          "libs/etc/cisstRobot/WAM/wam7.rob",
			          Rtw0, 
			          Kp, 
			          Kd,
				  "datafileL2.txt" );
  taskManager->AddComponent( &controllerVL );

  devImpedanceTunable controllerVR( "controllerVR",	// variable impedance controller for user input
			          0.002,
                                  devController::DISABLED,// initial state
			          OSA_CPU4,
		 	          "libs/etc/cisstRobot/WAM/wam7.rob",
			          Rtw0, 
			          Kp, 
			          Kd,
				  "datafileR2.txt" );
  taskManager->AddComponent( &controllerVR );


  taskManager->Connect( keyboard.GetName(),   "NextSetPointR",
			setpointsR.GetName(),  devSetPoints::Control );

  taskManager->Connect( keyboard.GetName(),   "NextSetPointL",
			setpointsL.GetName(),  devSetPoints::Control );

  taskManager->Connect( keyboard.GetName(),     "DisableConstantImpedanceR",
                        controllerCR.GetName(),  devTrajectory::Control );

  taskManager->Connect( keyboard.GetName(),     "DisableConstantImpedanceL",
                        controllerCL.GetName(),  devTrajectory::Control );

  taskManager->Connect( keyboard.GetName(),     "EnableVariableImpedanceR",
                        controllerVR.GetName(),  devTrajectory::Control );

  taskManager->Connect( keyboard.GetName(),     "EnableVariableImpedanceL",
                        controllerVL.GetName(),  devTrajectory::Control );

  taskManager->Connect( keyboard.GetName(),   "DisableTrajectoryR",
                        trajectoryR.GetName(), devTrajectory::Control );

  taskManager->Connect( keyboard.GetName(),   "DisableTrajectoryL",
                        trajectoryL.GetName(), devTrajectory::Control );

  taskManager->Connect( keyboard.GetName(),   "EnableVelEqnR",
                        VelEqnR.GetName(), devTrajectory::Control );

  taskManager->Connect( keyboard.GetName(),   "EnableFinL",
                        ForceInputL.GetName(), devTrajectory::Control );

  taskManager->Connect( keyboard.GetName(),   "ctrlenableR",
                        controllerCR.GetName(), devController::Control );

  taskManager->Connect( keyboard.GetName(),   "ctrlenableL",
                        controllerCL.GetName(), devController::Control );

  taskManager->Connect( trajectoryR.GetName(), devTrajectory::Input,
			setpointsR.GetName(),  devSetPoints::Output );

  taskManager->Connect( trajectoryL.GetName(), devTrajectory::Input,
			setpointsL.GetName(),  devSetPoints::Output );

  taskManager->Connect( trajectoryR.GetName(), devTrajectory::Output,
			controllerCR.GetName(), devController::Input );

  taskManager->Connect( trajectoryL.GetName(), devTrajectory::Output,
			controllerCL.GetName(), devController::Input );

  taskManager->Connect( das6014.GetName(),    devDAS6014::Output,
                        ForceInputL.GetName(), "Input" );

  taskManager->Connect( das6014.GetName(),    devDAS6014::Output,
                        controllerVR.GetName(), "Input2" );

  taskManager->Connect( das6014.GetName(),    devDAS6014::Output,
                        controllerVL.GetName(), "Input2" );

  taskManager->Connect( VelEqnR.GetName(), devTrajectory::Output,
                        controllerVR.GetName(), devController::Input );

  taskManager->Connect( ForceInputL.GetName(), devTrajectory::Output,
                        controllerVL.GetName(), devController::Input );

  taskManager->Connect( controllerCR.GetName(), devController::Output,
			wamR.GetName(),        devManipulator::Input );

  taskManager->Connect( controllerCL.GetName(), devController::Output,
			wamL.GetName(),        devManipulator::Input );

  taskManager->Connect( controllerCR.GetName(), devController::Feedback,
			wamR.GetName(),        devManipulator::Output );

  taskManager->Connect( controllerCL.GetName(), devController::Feedback,
			wamL.GetName(),        devManipulator::Output );

  taskManager->Connect( controllerVR.GetName(), devController::Output,
			wamR.GetName(),        devManipulator::Input );

  taskManager->Connect( controllerVL.GetName(), devController::Output,
			wamL.GetName(),        devManipulator::Input );

  taskManager->Connect( controllerVR.GetName(), devController::Feedback,
			wamR.GetName(),        devManipulator::Output );

  taskManager->Connect( controllerVL.GetName(), devController::Feedback,
			wamL.GetName(),        devManipulator::Output );

  // connect WAM output to trajectory feedback inputs, if needed
  taskManager->Connect( ForceInputL.GetName(),
                        devForceInput::WamStateInput,
                        wamL.GetName(),
                        devManipulator::Output );

  taskManager->Connect( VelEqnR.GetName(),
                        devVelEqn::WamStateInput,
                        wamR.GetName(),
                        devManipulator::Output );

  taskManager->CreateAll();


  keyboard.Start();
  setpointsR.Start();
  setpointsL.Start();
  wamR.Start();
  wamL.Start();
  osaSleep(0.1);
  controllerCR.Start();
  controllerCL.Start();
  controllerVR.Start();
  controllerVL.Start();
  osaSleep(0.1);
  trajectoryR.Start();
  trajectoryL.Start();
  osaSleep(0.1);
  das6014.Start();
  osaSleep(0.1);
  VelEqnR.Start();
  ForceInputL.Start();


//  taskManager->StartAll();
  pause();

  return 0;
}
