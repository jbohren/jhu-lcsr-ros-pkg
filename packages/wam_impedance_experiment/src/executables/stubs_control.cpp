#include <ros/ros.h>
#include <ros/package.h>

#include <cisstCommon/cmnConstants.h>

#include "wam_impedance_experiment/controllers/devMultipleControllers.h"
#include "wam_impedance_experiment/trajectories/devMultipleTrajectories.h"
#include "wam_impedance_experiment/devices/devStub.h"

#include <cisstMultiTask/mtsTaskManager.h>
#include <cisstVector/cisstVector.h>

#include <cisst_ros_integration/ros_tasks.h>

#include <cisstOSAbstraction/osaGetTime.h>

#include <sys/mman.h>

#include <boost/assign/std/vector.hpp>
#include <boost/assign/list_of.hpp>

#include <std_msgs/Empty.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float64MultiArray.h>
#include <wam_impedance_experiment/ControllerInfo.h>
#include <wam_impedance_experiment/TrajectoryInfo.h>
#include <wam_impedance_experiment/DAQinput.h>
#include <wam_impedance_experiment/ControllerData.h>
#include <wam_impedance_experiment/ProjectileInfo.h>
#include <sensor_msgs/JointState.h>

std::string robot_name = "robot0";

void wam_joint_state_cisst_to_ros(
    const mtsVector<double> & joint_angles,
    sensor_msgs::JointState & msg,
    std::string name)
{
  size_t n_joints = joint_angles.size();

  ROS_DEBUG_STREAM("Received "<<n_joints<<" joint angles...");
  using namespace boost::assign;

  ros::Time t(osaGetTime());
  msg.header.stamp = t;

  /*
  msg.name +=
    "DarmSim/RightArm/WAM/YawJoint",
    "DarmSim/RightArm/WAM/ShoulderPitchJoint",
    "DarmSim/RightArm/WAM/ShoulderYawJoint",
    "DarmSim/RightArm/WAM/ElbowJoint",
    "DarmSim/RightArm/WAM/UpperWristYawJoint",
    "DarmSim/RightArm/WAM/UpperWristPitchJoint",
    "DarmSim/RightArm/WAM/LowerWristYawJoint";
    */
  msg.name += name.c_str();

  msg.position.resize(n_joints);
  for(size_t i=0; i<n_joints; i++) {
    msg.position[i] = joint_angles[i];
  }

  msg.velocity += 0,0,0,0,0,0,0;
  msg.effort += 0,0,0,0,0,0,0;
}

void wam_controller_data_cisst_to_ros(
    ControllerData data,
    wam_impedance_experiment::ControllerData & msg,
    std::string name)
{
  ros::Time t_ros(data.t);
  ros::Duration dt_ros(data.dt);
  msg.t = t_ros;
  msg.name = name.c_str();
  msg.deltat = dt_ros;
  msg.position.resize(7);
  msg.velocity.resize(7);
  msg.setpointRnvelocity.resize(7);
  msg.setpointRn.resize(7);
  msg.setpoint_null.resize(7);
  msg.torque_gc.resize(7);
  msg.torque_ct.resize(7);
  msg.torque_imp.resize(7);
  msg.torque_imp_null.resize(7);
  msg.torque_imp_null_fric.resize(7);
  msg.torque_external.resize(7);
  msg.torque.resize(7);
  msg.setpointSE3velocity.resize(6);
  msg.Kp_Rn.resize(7);
  msg.Kd_Rn.resize(7);
  msg.Kp_SE3.resize(6);
  msg.Kd_SE3.resize(6);
  msg.erroraxis.resize(3);
  for (int i=0;i<7;i++) {
    msg.position[i] = data.q[i];
    msg.velocity[i] = data.qd[i];
    msg.setpointRnvelocity[i] = data.qsd[i];
    msg.setpointRn[i] = data.qs[i];
    msg.setpoint_null[i] = data.setpoint_null[i];
    msg.torque_gc[i] = data.tau_gc[i];
    msg.torque_ct[i] = data.tau_ct[i];
    msg.torque_imp[i] = data.tau_imp[i];
    msg.torque_imp_null[i] = data.tau_imp_null[i];
    msg.torque_imp_null_fric[i] = data.tau_imp_null_fric[i];
    msg.torque_external[i] = data.tau_external[i];
    msg.torque[i] = data.tau[i];
    msg.errorangle = data.theta;
    if (i<3) {
      msg.erroraxis[i] = data.erroraxis[i];
    }
    if (i<6) {
      msg.setpointSE3velocity[i] = data.xds[i];
      msg.Kp_SE3[i] = data.Kp_SE3[i];
      msg.Kd_SE3[i] = data.Kd_SE3[i];
    }
    msg.Kp_Rn[i] = data.Kp_Rn[i];
    msg.Kd_Rn[i] = data.Kd_Rn[i];
  }
  msg.Kp_null = data.Kp_null;
  msg.Kd_null = data.Kd_null;
  msg.m_y = data.m_y;
  msg.friction_gain = data.friction_gain;
  msg.impulse = data.impulse;
  msg.forceExternal = data.forceExternal;
  msg.controller = data.controllerNumber;
  msg.pose.layout.data_offset = 0;
  msg.pose.layout.dim.resize(2);
  msg.pose.layout.dim[0].stride = 4;
  msg.pose.layout.dim[1].stride = 1;
  msg.pose.data.resize(16);
  msg.setpointSE3.layout.data_offset = 0;
  msg.setpointSE3.layout.dim.resize(2);
  msg.setpointSE3.layout.dim[0].stride = 4;
  msg.setpointSE3.layout.dim[1].stride = 1;
  msg.setpointSE3.data.resize(16);
  msg.M.layout.data_offset = 0;
  msg.M.layout.dim.resize(2);
  msg.M.layout.dim[0].stride = 3;
  msg.M.layout.dim[1].stride = 1;
  msg.M.data.resize(9);
  msg.M_OS.layout.data_offset = 0;
  msg.M_OS.layout.dim.resize(2);
  msg.M_OS.layout.dim[0].stride = 6;
  msg.M_OS.layout.dim[1].stride = 1;
  msg.M_OS.data.resize(36);
  msg.M_JS.layout.data_offset = 0;
  msg.M_JS.layout.dim.resize(2);
  msg.M_JS.layout.dim[0].stride = 7;
  msg.M_JS.layout.dim[1].stride = 1;
  msg.M_JS.data.resize(49);
  for (int i=0;i<7;i++) {
    for (int j=0;j<7;j++) {
      if ((i<3) && (j<3)) {
        msg.M.data[(msg.M.layout.data_offset) + (msg.M.layout.dim[0].stride)*i + j] = data.M[i][j];
      }
      if ((i<4) && (j<4)) {
        msg.pose.data[(msg.pose.layout.data_offset) + (msg.pose.layout.dim[0].stride)*i + j] = data.Rtwn[i][j];
        msg.setpointSE3.data[(msg.setpointSE3.layout.data_offset) + (msg.setpointSE3.layout.dim[0].stride)*i + j] = data.Rtwns[i][j];
      }
      if ((i<6) && (j<6)) {
        msg.M_OS.data[(msg.M_OS.layout.data_offset) + (msg.M_OS.layout.dim[0].stride)*i + j] = data.M_OS[i][j];
      }
      msg.M_JS.data[(msg.M_JS.layout.data_offset) + (msg.M_JS.layout.dim[0].stride)*i + j] = data.M_JS[i][j];
    }
  }

  msg.tracking_goal = data.tracking_goal;
}

void wam_trajectory_cisst_to_ros(
    const TrajInfo & data,
    wam_impedance_experiment::TrajectoryInfo & msg,
    std::string name)
{
  ros::Time t_ros(data.t);
  msg.t = t_ros;
  msg.name = name.c_str();
  msg.trajectoryType = data.trajectoryNumber;
  msg.direction = 0;
  msg.setpointRn.resize(data.setpointRn.size());
  for (int i=0;i<(int)(data.setpointRn.size());i++) {
    msg.setpointRn[i] = data.setpointRn[i];
  }
  msg.setpointSE3.layout.data_offset = 0;
  msg.setpointSE3.layout.dim.resize(2);
  msg.setpointSE3.layout.dim[0].stride = 4;
  msg.setpointSE3.layout.dim[1].stride = 1;
  msg.setpointSE3.data.resize(16);
  for (int i=0;i<4;i++) {
    for (int j=0;j<4;j++) {
      msg.setpointSE3.data[(msg.setpointSE3.layout.data_offset) + (msg.setpointSE3.layout.dim[0].stride)*i + j] = data.setpointSE3[i][j];
    }
  }
  for (int i=0;i<(int)(data.velocitySE3.size());i++) {
    msg.velocitySE3[i] = data.velocitySE3[i];
  }
  msg.maxCommandSpeed = data.maxCommandSpeed;
}

void controller_info_ros_to_cisst(
    const wam_impedance_experiment::ControllerInfo & msg,
    ControlInfo & data)
{
  data.controllerNumber = msg.controllerType;
  data.stiffness = msg.stiffness;
  data.damping = msg.damping;
  data.setpoint_null.resize(7);
  for (int i=0;i<7;i++) {
    data.setpoint_null[i] = msg.setpoint_null[i];
  }
}

void gains_ros_to_cisst(
    const std_msgs::Float64MultiArray & msg,
    mtsVector<double> & data)
{
  data.resize(29);
  for (int i=0;i<29;i++) {
    data[i] = msg.data[i];
  }
}

void projectiles_ros_to_cisst(
    const wam_impedance_experiment::ProjectileInfo& msg,
    double & data)
{
  data = msg.impulse;
}

void tracking_goal_ros_to_cisst(
    const std_msgs::Float64& msg,
    double & data)
{
  data = msg.data;
}

void trajectory_info_ros_to_cisst(
    const wam_impedance_experiment::TrajectoryInfo & msg,
    TrajInfo & data)
{
  data.t = msg.t.toSec();
  data.trajectoryNumber = msg.trajectoryType;
  data.direction = msg.direction;
  data.setpointRn.SetSize(msg.setpointRn.size());
  data.setpointRn.SetAll(0.0);
  for (int i=0;i<(int)(msg.setpointRn.size());i++) {
    data.setpointRn[i] = msg.setpointRn[i];
  }
  for (int i=0;i<4;i++) {
    for (int j=0;j<4;j++) {
      data.setpointSE3[i][j] = msg.setpointSE3.data[(msg.setpointSE3.layout.data_offset) + (msg.setpointSE3.layout.dim[0].stride)*i + j];
    }
  }
}

void user_input_ros_to_cisst(
    const wam_impedance_experiment::DAQinput & msg,
    double & value)
{
  value = msg.force1;
}


void Quit(const std_msgs::Empty::ConstPtr& msg) {
  ROS_INFO("stubs_control: Quit message received.");
  ros::shutdown();
  exit(0);
}


int main(int argc, char** argv){

  //mlockall(MCL_CURRENT | MCL_FUTURE);
  //RT_TASK task;
  //rt_task_shadow( &task, "main", 60, T_FPU );

  ros::init(argc, argv, "stubs_demo");
  ros::NodeHandle n;

  // subscriber for receiving quit command
  ros::Subscriber sub_quit = n.subscribe<std_msgs::Empty>("quit", 50, Quit);

  //cmnLogger::SetMask(CMN_LOG_ALLOW_ALL);
  //cmnLogger::SetMaskDefaultLog(CMN_LOG_ALLOW_ALL);
  //cmnLogger::SetMaskFunction(CMN_LOG_ALLOW_ALL);
  //cmnLogger::SetMaskClassAll(CMN_LOG_ALLOW_ALL);


  mtsTaskManager* taskManager = mtsTaskManager::GetInstance();

  vctDynamicVector<double> qinit(7, 0.0);
  qinit[1] = -cmnPI_2;
  qinit[3] =  cmnPI;

  ROS_INFO("Creating stub...");

  devStub stub( "Stub", 0.02, OSA_CPU1, qinit );
  taskManager->AddComponent( &stub );

  ROS_INFO_STREAM("Getting robot description file...");

  /*
  std::string robfile("libs/etc/cisstRobot/WAM/wam7.rob");
  robfile = ros::package::getPath("cisst")+"/build/source/"+robfile;
  */
  std::string robfile("/include/wam7_AAB.rob");
  robfile = ros::package::getPath("wam_impedance_experiment")+robfile;

  ROS_INFO_STREAM("Got robot description file: \""<<robfile<<"\"");

  vctMatrixRotation3<double> Rw0(
      0.0,  0.0, -1.0, 
      0.0,  1.0,  0.0, 
      1.0,  0.0,  0.0 );
  vctFixedSizeVector<double,3> tw0(0.0);
  vctFrame4x4<double> Rtw0( Rw0, tw0 );

  // set initial gains for jointspace computed torque controller
  vctDynamicMatrix<double> Kp_Rn(7,7,0.0), Kd_Rn(7,7,0.0);
  Kp_Rn[0][0] = 1200.0;    Kd_Rn[0][0] = 20.0;
  Kp_Rn[1][1] = 800.0;     Kd_Rn[1][1] = 6.0;
  Kp_Rn[2][2] = 800.0;     Kd_Rn[2][2] = 20.0;
  Kp_Rn[3][3] = 800.0;     Kd_Rn[3][3] = 6.0;
  Kp_Rn[4][4] = 10000.0;   Kd_Rn[4][4] = 50.0;
  Kp_Rn[5][5] = 500.0;     Kd_Rn[5][5] = 10.0;
  Kp_Rn[6][6] = 8000.0;    Kd_Rn[6][6] = 30.0;

  // set initial gains for Cartesian space impedance controller
  vctFixedSizeMatrix<double,6,6> Kp_SE3(0.0), Kd_SE3(0.0);
  Kp_SE3[0][0] = 1000;     Kd_SE3[0][0] = 10.0;
  Kp_SE3[1][1] = 1000;     Kd_SE3[1][1] = 10.0;
  Kp_SE3[2][2] = 1000;     Kd_SE3[2][2] = 10.0;
  Kp_SE3[3][3] = 5.5;      Kd_SE3[3][3] = 0.1;
  Kp_SE3[4][4] = 1.0;      Kd_SE3[4][4] = 0.1;
  Kp_SE3[5][5] = 4.0;      Kd_SE3[5][5] = 0.1;

  ROS_INFO_STREAM("Creating controller...");

  devMultipleControllers controllers(
      "controllers", 
      0.002,
      devController::DISABLED,
      OSA_CPU2,
      robfile,
      Rtw0,
      Kp_Rn,
      Kd_Rn,
      Kp_SE3,
      Kd_SE3 );
  taskManager->AddComponent( &controllers );

  ROS_INFO_STREAM("Controller created.");

  ROS_INFO_STREAM("Creating trajectory...");

  devMultipleTrajectories trajectories(
      "trajectories",
      0.002,
      devTrajectory::ENABLED,
      OSA_CPU3,
      robfile,
      Rtw0,
      devTrajectory::POSITION | devTrajectory::VELOCITY,
      qinit,
      false); // for stubs, don't use actual position measurements
  taskManager->AddComponent( &trajectories );

  ROS_INFO_STREAM("Trajectory created.");


  // Add ROS interface
  ROS_INFO_STREAM("Adding ROS interfaces...");

  // Create an mtsRosPublisher that publishes data on the topic "joint_states"
  const double publish_period = 100 * cmn_ms; // in milliseconds
  mtsRosTaskPeriodic * ros_periodic = new mtsRosTaskPeriodic("ROS_PERIODIC_ADAPTER",publish_period);

  FcnReadPublisherAdapter<sensor_msgs::JointState, mtsVector<double> > joint_state_adapter(
      "ReadRnPosition",
      "/joint_states",50,
      ros::NodeHandle(),
      boost::bind(wam_joint_state_cisst_to_ros, _1, _2, robot_name) );

  ros_periodic->add_publisher(joint_state_adapter);
  taskManager->AddComponent( ros_periodic );

  // publisher for all sorts of data from controller
  mtsRosTaskFromSignal * ros_controller_data_pub = new mtsRosTaskFromSignal("ROS_CONTROLLER_DATA_PUB_ADAPTER");
  
  CmdWritePublisherAdapter<wam_impedance_experiment::ControllerData, ControllerData > controller_data_adapter(
      "ControllerDataPublishFunction",
      "/controller_data",50,
      ros::NodeHandle(),
      boost::bind(wam_controller_data_cisst_to_ros, _1, _2, robot_name) );

  ros_controller_data_pub->add_publisher(controller_data_adapter);
  taskManager->AddComponent( ros_controller_data_pub );

  // publisher for commanded trajectory
  mtsRosTaskFromSignal * ros_trajectory_pub = new mtsRosTaskFromSignal("ROS_TRAJECTORY_PUB_ADAPTER");

  CmdWritePublisherAdapter<wam_impedance_experiment::TrajectoryInfo, TrajInfo> joint_trajectory_adapter(
      "TrajectoryPublishFunction",
      "/trajectory",50,
      ros::NodeHandle(),
      boost::bind(wam_trajectory_cisst_to_ros, _1, _2, robot_name) );

  ros_trajectory_pub->add_publisher(joint_trajectory_adapter);
  taskManager->AddComponent( ros_trajectory_pub );

  // subscriber for control enable signal to controller
  mtsRosTaskFromSignal * ros_ctrlenable_cb = new mtsRosTaskFromSignal("ROS_CTRLENABLE_CB_ADAPTER");

  FcnVoidSubscriberAdapter<std_msgs::Empty> control_enable_adapter(
      devController::Enable,
      "/ctrl_enable",50);

  ros_ctrlenable_cb->add_subscriber(control_enable_adapter);
  taskManager->AddComponent( ros_ctrlenable_cb );

  // subscriber for controller info
  mtsRosTaskFromSignal * ros_ctrlinfo_cb = new mtsRosTaskFromSignal("ROS_CTRLINFO_CB_ADAPTER");

  FcnWriteSubscriberAdapter<wam_impedance_experiment::ControllerInfo, ControlInfo> control_info_adapter(
      "ControlInfoCommand",
      "/ctrl_info_0",50,
      ros::NodeHandle(),
      &controller_info_ros_to_cisst);

  ros_ctrlinfo_cb->add_subscriber(control_info_adapter);
  taskManager->AddComponent( ros_ctrlinfo_cb );

  // subscriber for gains
  mtsRosTaskFromSignal * ros_gains_cb = new mtsRosTaskFromSignal("ROS_GAINS_CB_ADAPTER");

  FcnWriteSubscriberAdapter<std_msgs::Float64MultiArray, mtsVector<double> > gains_adapter(
      "GainsCommand",
      "/gains_0",50,
      ros::NodeHandle(),
      &gains_ros_to_cisst);

  ros_gains_cb->add_subscriber(gains_adapter);
  taskManager->AddComponent( ros_gains_cb  );

  // subscriber for projectile info
  mtsRosTaskFromSignal * ros_projectiles_cb = new mtsRosTaskFromSignal("ROS_PROJECTILES_CB_ADAPTER");

  FcnWriteSubscriberAdapter<wam_impedance_experiment::ProjectileInfo, double> projectiles_adapter(
      "ImpulseCommand",
      "/projectiles",50,
      ros::NodeHandle(),
      &projectiles_ros_to_cisst);

  ros_projectiles_cb->add_subscriber(projectiles_adapter);
  taskManager->AddComponent( ros_projectiles_cb  );

  // subscriber getting tracking goal
  mtsRosTaskFromSignal * ros_tracking_goal_cb = new mtsRosTaskFromSignal("ROS_TRACKING_GOAL_CB_ADAPTER");

  FcnWriteSubscriberAdapter<std_msgs::Float64, double> tracking_goal_adapter(
      "TrackingGoalCommand",
      "/tracking_goal",50,
      ros::NodeHandle(),
      &tracking_goal_ros_to_cisst);

  ros_tracking_goal_cb->add_subscriber(tracking_goal_adapter);
  taskManager->AddComponent( ros_tracking_goal_cb  );

  // subscriber for trajectory info
  mtsRosTaskFromSignal * ros_trajinfo_cb = new mtsRosTaskFromSignal("ROS_TRAJINFO_CB_ADAPTER");

  FcnWriteSubscriberAdapter<wam_impedance_experiment::TrajectoryInfo, TrajInfo> traj_info_adapter(
      "TrajectoryInfoCommand",
      "/traj_info_0",50,
      ros::NodeHandle(),
      &trajectory_info_ros_to_cisst);

  ros_trajinfo_cb->add_subscriber(traj_info_adapter);
  taskManager->AddComponent( ros_trajinfo_cb );

  // subscriber for user input to pass to trajectory
  mtsRosTaskFromSignal * ros_userinput_cb = new mtsRosTaskFromSignal("ROS_USERINPUT_CB_ADAPTER");

  FcnWriteSubscriberAdapter<wam_impedance_experiment::DAQinput, double> user_input_adapter(
      "UserInputCommand",
      "/daq_input",50,
      ros::NodeHandle(),
      &user_input_ros_to_cisst);

  ros_userinput_cb->add_subscriber(user_input_adapter);
  taskManager->AddComponent( ros_userinput_cb );


  ROS_INFO_STREAM("ROS interfaces created.");


  // Connect everything
  
  taskManager->Connect(
      "ROS_CTRLENABLE_CB_ADAPTER",  "RequiredAdapter",
      controllers.GetName(),        devController::Control );

  taskManager->Connect(
      "ROS_CTRLINFO_CB_ADAPTER",  "RequiredAdapter",
      controllers.GetName(),      "ControlInfo" );

  taskManager->Connect(
      "ROS_GAINS_CB_ADAPTER",  "RequiredAdapter",
      controllers.GetName(),      "Gains" );

  taskManager->Connect(
      "ROS_TRACKING_GOAL_CB_ADAPTER",  "RequiredAdapter",
      controllers.GetName(),         "TrackingGoal" );

  taskManager->Connect(
      "ROS_PROJECTILES_CB_ADAPTER",  "RequiredAdapter",
      controllers.GetName(),         "Impulse" );

  taskManager->Connect(
      "ROS_TRAJINFO_CB_ADAPTER",  "RequiredAdapter",
      trajectories.GetName(),     "TrajectoryInfo" );

  taskManager->Connect(
      "ROS_USERINPUT_CB_ADAPTER", "RequiredAdapter",
      trajectories.GetName(),     "UserInput" );

  taskManager->Connect(
      controllers.GetName(),  "InputRn",
      trajectories.GetName(), "OutputRn");

  taskManager->Connect(
      controllers.GetName(),  "InputSE3",
      trajectories.GetName(), "OutputSE3");

  /*
  taskManager->Connect(
      trajectories.GetName(),  "TrajectoryResetDoneInterface",
      controllers.GetName(),   "TrajectoryResetDoneInterface");

  taskManager->Connect(
      controllers.GetName(),  "TrajectoryResetInterface",
      trajectories.GetName(), "TrajectoryResetInterface");
      */

  taskManager->Connect(
      controllers.GetName(),  devController::Output,
      stub.GetName(),         devManipulator::Input );

  taskManager->Connect(
      controllers.GetName(),  devController::Feedback,
			stub.GetName(),         devManipulator::Output );

  taskManager->Connect(
      trajectories.GetName(),   "WAM_Feedback",
			stub.GetName(),           devManipulator::Output );

  taskManager->Connect(
      "ROS_PERIODIC_ADAPTER",  "RequiredAdapter",
			stub.GetName(),  devManipulator::Output );

  taskManager->Connect(
      "ROS_CONTROLLER_DATA_PUB_ADAPTER",  "ProvidedAdapter",
      controllers.GetName(),   "ControllerDataPublisher" );

  taskManager->Connect(
      "ROS_TRAJECTORY_PUB_ADAPTER", "ProvidedAdapter",
      trajectories.GetName(),       "TrajectoryPublisher" );

  taskManager->CreateAll();
  taskManager->StartAll();

  ros::spin();

  return 0;
}
