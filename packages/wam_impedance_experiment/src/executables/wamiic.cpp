#include <cisstDevices/can/devRTSocketCAN.h>

#include <cisstCommon/cmnConstants.h>

#include "wam_impedance_experiment/devices/devKeyboardAAB.h"

// To use a trajectory
#include <cisstDevices/robotcomponents/trajectories/devSetPoints.h>
#include <cisstDevices/robotcomponents/trajectories/devLinearSE3.h>

#include "wam_impedance_experiment/robotcomponents/devWAM_AAB.h"

// To use a control loop
#include "wam_impedance_experiment/controllers/devImpedanceIC.h"

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
  taskManager->AddComponent( &keyboard );

  devRTSocketCAN can( "rtcan0", devCAN::RATE_1000 );

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

/*
  vector<string> geomfiles;
  string path("libs/etc/cisstRobot/WAM/");
  geomfiles.push_back( path + "l1.obj" );
  geomfiles.push_back( path + "l2.obj" );
  geomfiles.push_back( path + "l3.obj" );
  geomfiles.push_back( path + "l4.obj" );
  geomfiles.push_back( path + "l5.obj" );
  geomfiles.push_back( path + "l6.obj" );
  geomfiles.push_back( path + "l7.obj" );
*/

  // Create a set point generator
  std::vector< vctFrame4x4<double> > Rtwn;

  vctFixedSizeVector<double,3> t0 = Rtwninit.Translation();
  vctFixedSizeVector<double,3> t1 = t0;
  t1[0] = -0.85;
  t1[2] = 0.5;
  vctFixedSizeVector<double,3> t2 = t1;
  vctFixedSizeVector<double,3> t3 = t2;
  vctFixedSizeVector<double,3> t4 = t3;
  t4[1] = 0.4;
  vctFixedSizeVector<double,3> t5 = t4;
  t5[1] = -0.4;
  vctFixedSizeVector<double,3> t6 = t4;
  t6[2] = 0.8;
  vctFixedSizeVector<double,3> t7 = t5;
  t7[2] = 0.8;
  vctFixedSizeVector<double,3> t8 = t6;
  t8[0] = -1;
  vctFixedSizeVector<double,3> t9 = t7;
  t9[0] = -1;

/*
  vctFixedSizeVector<double,3> t0 = Rtwninit.Translation();
  vctFixedSizeVector<double,3> t1 = t0;
  t1[0] = t1[0] - 0.85;
  vctFixedSizeVector<double,3> t2 = t1;
  t2[0] = t2[0] + 0.3;
  t2[1] = t2[1] + 0.2;
  vctFixedSizeVector<double,3> t3 = t2;
  t3[1] = t3[1] - 0.2;
  vctFixedSizeVector<double,3> t4 = t3;
  t4[2] = t4[2] + 0.2;
  t4[0] = t4[0] - 0.2;
*/

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

  vctFrame4x4<double> Rtwn1( RwR, t1);
  vctFrame4x4<double> Rtwn2( RwR, t2);
  vctFrame4x4<double> Rtwn3( RwR, t3);
  vctFrame4x4<double> Rtwn4( RwR, t4);
  vctFrame4x4<double> Rtwn5( RwR, t5);
  vctFrame4x4<double> Rtwn6( RwR, t6);
  vctFrame4x4<double> Rtwn7( RwR, t7);
  vctFrame4x4<double> Rtwn8( RwR, t8);
  vctFrame4x4<double> Rtwn9( RwR, t9);
/*
  vctFrame4x4<double> Rtwn1( RwR, t1);
  vctFrame4x4<double> Rtwn2( RwR, t2);
  vctFrame4x4<double> Rtwn3( RwR, t3);
  vctFrame4x4<double> Rtwn4( RwR, t4);
  vctFrame4x4<double> Rtwn5( RwR, t5);
*/
  Rtwn.push_back( Rtwninit );
  Rtwn.push_back( Rtwn1 );
//  Rtwn.push_back( Rtwn2 );
//  Rtwn.push_back( Rtwn3 );
  Rtwn.push_back( Rtwn4 );
  Rtwn.push_back( Rtwn5 );
/*
  Rtwn.push_back( Rtwn6 );
  Rtwn.push_back( Rtwn7 );
  Rtwn.push_back( Rtwn8 );
  Rtwn.push_back( Rtwn9 );
  Rtwn.push_back( Rtwn4 );
  Rtwn.push_back( Rtwn5 );
  Rtwn.push_back( Rtwn4 );
  Rtwn.push_back( Rtwn5 );
*/
  Rtwn.push_back( Rtwn4 );
  Rtwn.push_back( Rtwn5 );
  Rtwn.push_back( Rtwn4 );
//  Rtwn.push_back( Rtwn2 );
  Rtwn.push_back( Rtwn1 );
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

  // Create the controller
  vctFixedSizeMatrix<double,6,6> Kp(0.0), Kd(0.0), Ki(0.0);
  Kp[0][0] = 500;    Kd[0][0] = 10.0;
  Kp[1][1] = 40;    Kd[1][1] = 2.0;	Ki[1][1] = 3.0;
  Kp[2][2] = 500;    Kd[2][2] = 10.0;
  Kp[3][3] = 4.0;   Kd[3][3] = 0.1;
  Kp[4][4] = 1.0;   Kd[4][4] = 0.10;
  Kp[5][5] = 4.0;   Kd[5][5] = 0.1;
  devImpedanceIC controller( "controller",
			   0.002,
                           devController::DISABLED,// initial state
			   OSA_CPU3,
			   "libs/etc/cisstRobot/WAM/wam7.rob",
			   Rtw0, 
			   Ki, 
			   Kp, 
			   Kd,
			   "datafile.txt" );
  taskManager->AddComponent( &controller );

  /*
  // The WAM
  devODEManipulator WAM( "WAM",          // The task name "WAM"
			 0.001,          // The WAM runs at 200Hz
			 devTrajectory::ENABLED,// initial state
			 OSA_CPU4,
			 devManipulator::FORCETORQUE,
			 world,          // The world used to simulate the WAM
			 "libs/etc/cisstRobot/WAM/wam7.rob",
			 Rtw0,           // The transformation of the base
			 qinit,          // The initial joint positions
			 geomfiles );    // The geometries
  taskManager->AddComponent( &WAM );
  */

  taskManager->Connect( keyboard.GetName(),   "NextSetPoint",
			setpoints.GetName(),  devSetPoints::Control );

  taskManager->Connect( keyboard.GetName(), "ctrlenable",
                        controller.GetName(), devController::Control );

  taskManager->Connect( trajectory.GetName(), devTrajectory::Input,
			setpoints.GetName(),  devSetPoints::Output );

  taskManager->Connect( trajectory.GetName(), devTrajectory::Output,
			controller.GetName(), devController::Input );

  taskManager->Connect( controller.GetName(), devController::Output,
			wam.GetName(),        devManipulator::Input );

  taskManager->Connect( controller.GetName(), devController::Feedback,
			wam.GetName(),        devManipulator::Output );
  /*
  taskManager->Connect( trajectory.GetName(), devTrajectory::Output,
			WAM.GetName(), devManipulator::Input );
  */
  taskManager->CreateAll();


  keyboard.Start();
  setpoints.Start();
  wam.Start();
  osaSleep(0.1);
  trajectory.Start();
  osaSleep(0.1);
  controller.Start();
//  osaSleep(0.1);
//  world.Start();

//  taskManager->StartAll();
  pause();
  //glutMainLoop();

  return 0;
}
