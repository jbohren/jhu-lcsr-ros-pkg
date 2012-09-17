#include <cisstCommon/cmnConstants.h>

#include <ros/package.h>

#include "wam_impedance_experiment/devices/devKeyboardAAB.h"

// To use a trajectory
#include <cisstDevices/robotcomponents/trajectories/devSetPoints.h>
#include <cisstDevices/robotcomponents/trajectories/devLinearSE3.h>

#include "wam_impedance_experiment/devices/devStub.h"

// To use a control loop
#include "wam_impedance_experiment/controllers/devImpedance.h"

// To run the show
#include <cisstMultiTask/mtsTaskManager.h>
#include <cisstOSAbstraction/osaSleep.h>

using namespace std;

int main(int argc, char** argv){

  mtsTaskManager* taskManager = mtsTaskManager::GetInstance();

  devKeyboardAAB keyboard;
  keyboard.SetQuitKey('q');
  keyboard.AddKeyWriteFunction( 'n', "NextSetPoint", devSetPoints::NextSetPoint, true);
  keyboard.AddKeyVoidFunction( 'G', "ctrlenable", devController::Enable);
  taskManager->AddComponent( &keyboard );

  vctDynamicVector<double> qinit(7, 0.0);              // Initial joint values

  devStub stub( "Stub", 0.02, OSA_CPU1, qinit );
  taskManager->AddComponent( &stub );

  vctMatrixRotation3<double> Rw0(  0.0, 0.0, -1.0,
                                   0.0, 1.0, 0.0,
                                   1.0, 0.0, 0.0 );
  vctFixedSizeVector<double,3> tw0(0.0, 0.0, 1.0);
  vctFrame4x4<double> Rtw0(Rw0,tw0);                   // base transformation

  std::string robfile("libs/etc/cisstRobot/WAM/wam7.rob");
  robfile = ros::package::getPath("cisst")+"/build/source/"+robfile;

  // get initial end effector position and orientation in world frame
  robManipulator WAMtmp( robfile.c_str(), Rtw0 );
  vctFrame4x4<double> Rtwninit = WAMtmp.ForwardKinematics( qinit );

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
                           0.01,                  //The trajectory runs at 100Hz
                           devTrajectory::ENABLED,// initial state
			   OSA_CPU2,
                           devTrajectory::QUEUE,
			   devTrajectory::POSITION,
			   Rtwninit,
                           0.3,              // Max linear velocity
                           0.3 );            // Max angular velocity
  taskManager->AddComponent( &trajectory );

  // Create the controller
  vctFixedSizeMatrix<double,6,6> Kp(0.0), Kd(0.0);
  Kp[0][0] = 300;    Kd[0][0] = 10.0;
  Kp[1][1] = 40;    Kd[1][1] = 2.0;
  Kp[2][2] = 300;    Kd[2][2] = 10.0;
  Kp[3][3] = 4.0;   Kd[3][3] = 0.1;
  Kp[4][4] = 1.0;   Kd[4][4] = 0.1;
  Kp[5][5] = 4.0;   Kd[5][5] = 0.1;

  

  devImpedance controller( "controller",
			   0.02,
                           devController::DISABLED,// initial state
			   OSA_CPU3,
			   robfile.c_str(),
			   Rtw0, 
			   Kp, 
			   Kd,
			   "datafile.txt" );
  taskManager->AddComponent( &controller );


  taskManager->Connect( keyboard.GetName(),   "NextSetPoint",
			setpoints.GetName(),  devSetPoints::Control );

  taskManager->Connect( keyboard.GetName(), "ctrlenable",
                        controller.GetName(), devController::Control );

  taskManager->Connect( trajectory.GetName(), devTrajectory::Input,
			setpoints.GetName(),  devSetPoints::Output );

  taskManager->Connect( trajectory.GetName(), devTrajectory::Output,
			controller.GetName(), devController::Input );

  taskManager->Connect( controller.GetName(), devController::Output,
                        stub.GetName(),       devManipulator::Input );

  taskManager->Connect( controller.GetName(), devController::Feedback,
                        stub.GetName(),       devManipulator::Output );

  taskManager->CreateAll();

std::cout << "Components created.\n";

  keyboard.Start();
  setpoints.Start();
  stub.Start();
  osaSleep(0.1);
  trajectory.Start();
  osaSleep(0.1);
  controller.Start();
std::cout << "Components started.\n";

//controller.EnableCommand();
//  taskManager->StartAll();
  pause();

  return 0;
}
