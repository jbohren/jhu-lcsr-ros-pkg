#include <ros/ros.h>
#include <ros/package.h>

#include <cisstDevices/can/devRTSocketCAN.h>
#include "wam_impedance_experiment/robotcomponents/devWAM_AAB.h"

#include <cisstCommon/cmnConstants.h>

#include "wam_impedance_experiment/controllers/devMultipleControllers.h"
#include "wam_impedance_experiment/trajectories/devMultipleTrajectories.h"

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

#include <boost/bind.hpp>


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
  ROS_INFO("two_wam_control: Quit message received.");
  ros::shutdown();
  exit(0);
}


int main(int argc, char** argv){

  //mlockall(MCL_CURRENT | MCL_FUTURE);
  //RT_TASK task;
  //rt_task_shadow( &task, "main", 60, T_FPU );

  ros::init(argc, argv, "two_wam_control");
  ros::NodeHandle n;

  // subscriber for receiving quit command
  ros::Subscriber sub_quit = n.subscribe<std_msgs::Empty>("quit", 50, Quit);

  //cmnLogger::SetMask(CMN_LOG_ALLOW_ALL);
  //cmnLogger::SetMaskDefaultLog(CMN_LOG_ALLOW_ALL);
  //cmnLogger::SetMaskFunction(CMN_LOG_ALLOW_ALL);
  //cmnLogger::SetMaskClassAll(CMN_LOG_ALLOW_ALL);


  mtsTaskManager* taskManager = mtsTaskManager::GetInstance();

  // initial position, same for both wams
  vctDynamicVector<double> qinit(7, 0.0);
  qinit[1] = -cmnPI_2;
  qinit[3] =  cmnPI;

  ROS_INFO("Connecting to CAN socket 0...");
  devRTSocketCAN can0( "rtcan0", devCAN::RATE_1000 );
  ROS_INFO("Connecting to CAN socket 1...");
  devRTSocketCAN can1( "rtcan1", devCAN::RATE_1000 );

  ROS_INFO("Constructing WAM component (left)...");
  devWAM_AAB wamL( "WAM_L", 0.002, OSA_CPU1, &can0, qinit );
  ROS_INFO("Constructing WAM component (right)...");
  devWAM_AAB wamR( "WAM_R", 0.002, OSA_CPU1, &can1, qinit );

  ROS_INFO("Configuring WAM component (left)...");
  wamL.Configure();
  ROS_INFO("Configuring WAM component (right)...");
  wamR.Configure();

  taskManager->AddComponent( &wamL );
  taskManager->AddComponent( &wamR );

  ROS_INFO_STREAM("Getting robot description file...");

  /*
  std::string robfile("libs/etc/cisstRobot/WAM/wam7.rob");
  robfile = ros::package::getPath("cisst")+"/build/source/"+robfile;
  */

  std::string robfile0("/include/wam7_AAB.rob");
  robfile0 = ros::package::getPath("wam_impedance_experiment")+robfile0;
  ROS_INFO_STREAM("Got robot description file: \""<<robfile0<<"\"");

  std::string robfile1("/include/wam7ft_AAB.rob");
  robfile1 = ros::package::getPath("wam_impedance_experiment")+robfile1;
  ROS_INFO_STREAM("Got robot description file: \""<<robfile1<<"\"");

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
  Kp_SE3[0][0] = 1000;    Kd_SE3[0][0] = 10.0;
  Kp_SE3[1][1] = 1000;    Kd_SE3[1][1] = 10.0;
  Kp_SE3[2][2] = 1000;    Kd_SE3[2][2] = 10.0;
  Kp_SE3[3][3] = 1.0;    Kd_SE3[3][3] = 0.1;
  Kp_SE3[4][4] = 1.0;    Kd_SE3[4][4] = 0.1;
  Kp_SE3[5][5] = 1.0;    Kd_SE3[5][5] = 0.1;

  ROS_INFO_STREAM("Creating controllers...");

  devMultipleControllers controllersL(
      "controllersL", 
      0.002,
      devController::DISABLED,
      OSA_CPU2,
      robfile0,
      Rtw0,
      Kp_Rn,
      Kd_Rn,
      Kp_SE3,
      Kd_SE3 );
  taskManager->AddComponent( &controllersL );

  devMultipleControllers controllersR(
      "controllersR", 
      0.002,
      devController::DISABLED,
      OSA_CPU2,
      robfile1,
      Rtw0,
      Kp_Rn,
      Kd_Rn,
      Kp_SE3,
      Kd_SE3 );
  taskManager->AddComponent( &controllersR );

  ROS_INFO_STREAM("Controllers created.");

  ROS_INFO_STREAM("Creating trajectories...");

  devMultipleTrajectories trajectoriesL(
      "trajectoriesL",
      0.002,
      devTrajectory::ENABLED,
      OSA_CPU3,
      robfile0,
      Rtw0,
      devTrajectory::POSITION | devTrajectory::VELOCITY,
      qinit,
      true); // for wam, use actual position measurements when switching between trajectory types
  taskManager->AddComponent( &trajectoriesL );

  devMultipleTrajectories trajectoriesR(
      "trajectoriesR",
      0.002,
      devTrajectory::ENABLED,
      OSA_CPU3,
      robfile1,
      Rtw0,
      devTrajectory::POSITION | devTrajectory::VELOCITY,
      qinit,
      true); // for wam, use actual position measurements when switching between trajectory types
  taskManager->AddComponent( &trajectoriesR );

  ROS_INFO_STREAM("Trajectories created.");


  // Add ROS interface
  ROS_INFO_STREAM("Adding ROS interfaces...");

  // Create two mtsRosPublisher's that publish data on the topic "joint_states".
  // Create one for each WAM, but publish on the same topic.
  // Using the same topic makes it easier for the data logger.
  const double publish_period = 10 * cmn_ms; // in milliseconds
  mtsRosTaskPeriodic * ros_periodic_L = new mtsRosTaskPeriodic("ROS_PERIODIC_ADAPTER_L",publish_period);
  mtsRosTaskPeriodic * ros_periodic_R = new mtsRosTaskPeriodic("ROS_PERIODIC_ADAPTER_R",publish_period);

  FcnReadPublisherAdapter<sensor_msgs::JointState, mtsVector<double> > joint_state_adapter_L(
      "ReadRnPosition",
      "/joint_states",50,
      ros::NodeHandle(),
      boost::bind(wam_joint_state_cisst_to_ros, _1, _2, "left") );
      //&wam_joint_state_cisst_to_ros);

  FcnReadPublisherAdapter<sensor_msgs::JointState, mtsVector<double> > joint_state_adapter_R(
      "ReadRnPosition",
      "/joint_states",50,
      ros::NodeHandle(),
      boost::bind(wam_joint_state_cisst_to_ros, _1, _2, "right") );
      //&wam_joint_state_cisst_to_ros);

  ros_periodic_L->add_publisher(joint_state_adapter_L);
  ros_periodic_R->add_publisher(joint_state_adapter_R);
  taskManager->AddComponent( ros_periodic_L );
  taskManager->AddComponent( ros_periodic_R );

  // Create publishers for all sorts of data from controllers.
  // Create one for each WAM, but publish on the same topic.
  // Using the same topic makes it easier for the data logger.
  mtsRosTaskFromSignal * ros_controller_data_pub_L = new mtsRosTaskFromSignal("ROS_CONTROLLER_DATA_PUB_ADAPTER_L");
  mtsRosTaskFromSignal * ros_controller_data_pub_R = new mtsRosTaskFromSignal("ROS_CONTROLLER_DATA_PUB_ADAPTER_R");
  
  CmdWritePublisherAdapter<wam_impedance_experiment::ControllerData, ControllerData > controller_data_adapter_L(
      "ControllerDataPublishFunction",
      "/controller_data",50,
      ros::NodeHandle(),
      boost::bind(wam_controller_data_cisst_to_ros, _1, _2, "left") );

  CmdWritePublisherAdapter<wam_impedance_experiment::ControllerData, ControllerData > controller_data_adapter_R(
      "ControllerDataPublishFunction",
      "/controller_data",50,
      ros::NodeHandle(),
      boost::bind(wam_controller_data_cisst_to_ros, _1, _2, "right") );

  ros_controller_data_pub_L->add_publisher(controller_data_adapter_L);
  ros_controller_data_pub_R->add_publisher(controller_data_adapter_R);
  taskManager->AddComponent( ros_controller_data_pub_L );
  taskManager->AddComponent( ros_controller_data_pub_R );

  // Create publishers for commanded trajectories.
  // Create one for each WAM, but publish on the same topic.
  // Using the same topic makes it easier for the data logger.
  mtsRosTaskFromSignal * ros_trajectory_pub_L = new mtsRosTaskFromSignal("ROS_TRAJECTORY_PUB_ADAPTER_L");
  mtsRosTaskFromSignal * ros_trajectory_pub_R = new mtsRosTaskFromSignal("ROS_TRAJECTORY_PUB_ADAPTER_R");

  CmdWritePublisherAdapter<wam_impedance_experiment::TrajectoryInfo, TrajInfo> joint_trajectory_adapter_L(
      "TrajectoryPublishFunction",
      "/trajectory",50,
      ros::NodeHandle(),
      boost::bind(wam_trajectory_cisst_to_ros, _1, _2, "left") );
      //&wam_trajectory_cisst_to_ros);

  CmdWritePublisherAdapter<wam_impedance_experiment::TrajectoryInfo, TrajInfo> joint_trajectory_adapter_R(
      "TrajectoryPublishFunction",
      "/trajectory",50,
      ros::NodeHandle(),
      boost::bind(wam_trajectory_cisst_to_ros, _1, _2, "right") );
      //&wam_trajectory_cisst_to_ros);

  ros_trajectory_pub_L->add_publisher(joint_trajectory_adapter_L);
  ros_trajectory_pub_R->add_publisher(joint_trajectory_adapter_R);
  taskManager->AddComponent( ros_trajectory_pub_L );
  taskManager->AddComponent( ros_trajectory_pub_R );

  // Create subscriber for control enable signal to controllers.
  // The same message goes to both controllers.
  // Need two separate adapters, but both subscribe to the same topic.
  mtsRosTaskFromSignal * ros_ctrlenable_cb_L = new mtsRosTaskFromSignal("ROS_CTRLENABLE_CB_ADAPTER_L");
  mtsRosTaskFromSignal * ros_ctrlenable_cb_R = new mtsRosTaskFromSignal("ROS_CTRLENABLE_CB_ADAPTER_R");

  FcnVoidSubscriberAdapter<std_msgs::Empty> control_enable_adapter_L(
      devController::Enable,
      "/ctrl_enable",50);

  FcnVoidSubscriberAdapter<std_msgs::Empty> control_enable_adapter_R(
      devController::Enable,
      "/ctrl_enable",50);

  ros_ctrlenable_cb_L->add_subscriber(control_enable_adapter_L);
  ros_ctrlenable_cb_R->add_subscriber(control_enable_adapter_R);
  taskManager->AddComponent( ros_ctrlenable_cb_L );
  taskManager->AddComponent( ros_ctrlenable_cb_R );

  // Create subscribers for controller info.
  // There are separate topics for the two WAMs, so two adapters are needed.
  mtsRosTaskFromSignal * ros_ctrlinfo_cb_L = new mtsRosTaskFromSignal("ROS_CTRLINFO_CB_ADAPTER_L");
  mtsRosTaskFromSignal * ros_ctrlinfo_cb_R = new mtsRosTaskFromSignal("ROS_CTRLINFO_CB_ADAPTER_R");

  FcnWriteSubscriberAdapter<wam_impedance_experiment::ControllerInfo, ControlInfo> control_info_adapter_L(
      "ControlInfoCommand",
      "/ctrl_info_0",50,
      ros::NodeHandle(),
      &controller_info_ros_to_cisst);

  FcnWriteSubscriberAdapter<wam_impedance_experiment::ControllerInfo, ControlInfo> control_info_adapter_R(
      "ControlInfoCommand",
      "/ctrl_info_1",50,
      ros::NodeHandle(),
      &controller_info_ros_to_cisst);

  ros_ctrlinfo_cb_L->add_subscriber(control_info_adapter_L);
  ros_ctrlinfo_cb_R->add_subscriber(control_info_adapter_R);
  taskManager->AddComponent( ros_ctrlinfo_cb_L );
  taskManager->AddComponent( ros_ctrlinfo_cb_R );

  // Create subscribers for gains.
  // There are separate topics for the two WAMs, so two adapters are needed.
  mtsRosTaskFromSignal * ros_gains_cb_L = new mtsRosTaskFromSignal("ROS_GAINS_CB_ADAPTER_L");
  mtsRosTaskFromSignal * ros_gains_cb_R = new mtsRosTaskFromSignal("ROS_GAINS_CB_ADAPTER_R");

  FcnWriteSubscriberAdapter<std_msgs::Float64MultiArray, mtsVector<double> > gains_adapter_L(
      "GainsCommand",
      "/gains_0",50,
      ros::NodeHandle(),
      &gains_ros_to_cisst);

  FcnWriteSubscriberAdapter<std_msgs::Float64MultiArray, mtsVector<double> > gains_adapter_R(
      "GainsCommand",
      "/gains_1",50,
      ros::NodeHandle(),
      &gains_ros_to_cisst);

  ros_gains_cb_L->add_subscriber(gains_adapter_L);
  ros_gains_cb_R->add_subscriber(gains_adapter_R);
  taskManager->AddComponent( ros_gains_cb_L );
  taskManager->AddComponent( ros_gains_cb_R );

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

  // Create subscribers for trajectory info.
  // There are separate topics for the two WAMs, so two adapters are needed.
  mtsRosTaskFromSignal * ros_trajinfo_cb_L = new mtsRosTaskFromSignal("ROS_TRAJINFO_CB_ADAPTER_L");
  mtsRosTaskFromSignal * ros_trajinfo_cb_R = new mtsRosTaskFromSignal("ROS_TRAJINFO_CB_ADAPTER_R");

  FcnWriteSubscriberAdapter<wam_impedance_experiment::TrajectoryInfo, TrajInfo> traj_info_adapter_L(
      "TrajectoryInfoCommand",
      "/traj_info_0",50,
      ros::NodeHandle(),
      &trajectory_info_ros_to_cisst);

  FcnWriteSubscriberAdapter<wam_impedance_experiment::TrajectoryInfo, TrajInfo> traj_info_adapter_R(
      "TrajectoryInfoCommand",
      "/traj_info_1",50,
      ros::NodeHandle(),
      &trajectory_info_ros_to_cisst);

  ros_trajinfo_cb_L->add_subscriber(traj_info_adapter_L);
  ros_trajinfo_cb_R->add_subscriber(traj_info_adapter_R);
  taskManager->AddComponent( ros_trajinfo_cb_L );
  taskManager->AddComponent( ros_trajinfo_cb_R );

  // Create subscriber for user input to pass to trajectories.
  // Both stubs can use this information at the same time, but it comes from a single topic.
  // Need two separate adapters to connect to the two trajectories.
  mtsRosTaskFromSignal * ros_userinput_cb_L = new mtsRosTaskFromSignal("ROS_USERINPUT_CB_ADAPTER_L");
  mtsRosTaskFromSignal * ros_userinput_cb_R = new mtsRosTaskFromSignal("ROS_USERINPUT_CB_ADAPTER_R");

  FcnWriteSubscriberAdapter<wam_impedance_experiment::DAQinput, double> user_input_adapter_L(
      "UserInputCommand",
      "/daq_input",50,
      ros::NodeHandle(),
      &user_input_ros_to_cisst);

  FcnWriteSubscriberAdapter<wam_impedance_experiment::DAQinput, double> user_input_adapter_R(
      "UserInputCommand",
      "/daq_input",50,
      ros::NodeHandle(),
      &user_input_ros_to_cisst);

  ros_userinput_cb_L->add_subscriber(user_input_adapter_L);
  ros_userinput_cb_R->add_subscriber(user_input_adapter_R);
  taskManager->AddComponent( ros_userinput_cb_L );
  taskManager->AddComponent( ros_userinput_cb_R );


  ROS_INFO_STREAM("ROS interfaces created.");


  // Connect everything
  
  taskManager->Connect(
      "ROS_CTRLENABLE_CB_ADAPTER_L",  "RequiredAdapter",
      controllersL.GetName(),        devController::Control );

  taskManager->Connect(
      "ROS_CTRLENABLE_CB_ADAPTER_R",  "RequiredAdapter",
      controllersR.GetName(),        devController::Control );

  taskManager->Connect(
      "ROS_CTRLINFO_CB_ADAPTER_L",  "RequiredAdapter",
      controllersL.GetName(),       "ControlInfo" );

  taskManager->Connect(
      "ROS_CTRLINFO_CB_ADAPTER_R",  "RequiredAdapter",
      controllersR.GetName(),       "ControlInfo" );

  taskManager->Connect(
      "ROS_GAINS_CB_ADAPTER_L",  "RequiredAdapter",
      controllersL.GetName(),        "Gains" );

  taskManager->Connect(
      "ROS_GAINS_CB_ADAPTER_R",  "RequiredAdapter",
      controllersR.GetName(),        "Gains" );

  taskManager->Connect(
      "ROS_TRACKING_GOAL_CB_ADAPTER",  "RequiredAdapter",
      controllersL.GetName(),         "TrackingGoal" );

  taskManager->Connect(
      "ROS_PROJECTILES_CB_ADAPTER",  "RequiredAdapter",
      controllersL.GetName(),         "Impulse" );

  taskManager->Connect(
      "ROS_TRAJINFO_CB_ADAPTER_L",  "RequiredAdapter",
      trajectoriesL.GetName(),      "TrajectoryInfo" );

  taskManager->Connect(
      "ROS_TRAJINFO_CB_ADAPTER_R",  "RequiredAdapter",
      trajectoriesR.GetName(),      "TrajectoryInfo" );

  taskManager->Connect(
      "ROS_USERINPUT_CB_ADAPTER_L", "RequiredAdapter",
      trajectoriesL.GetName(),     "UserInput" );

  taskManager->Connect(
      "ROS_USERINPUT_CB_ADAPTER_R", "RequiredAdapter",
      trajectoriesR.GetName(),     "UserInput" );

  taskManager->Connect(
      controllersL.GetName(),  "InputRn",
      trajectoriesL.GetName(), "OutputRn");

  taskManager->Connect(
      controllersR.GetName(),  "InputRn",
      trajectoriesR.GetName(), "OutputRn");

  taskManager->Connect(
      controllersL.GetName(),  "InputSE3",
      trajectoriesL.GetName(), "OutputSE3");

  taskManager->Connect(
      controllersR.GetName(),  "InputSE3",
      trajectoriesR.GetName(), "OutputSE3");

  /*
  taskManager->Connect(
      trajectoriesL.GetName(),  "TrajectoryResetDoneInterface",
      controllersL.GetName(),   "TrajectoryResetDoneInterface");

  taskManager->Connect(
      trajectoriesR.GetName(),  "TrajectoryResetDoneInterface",
      controllersR.GetName(),   "TrajectoryResetDoneInterface");

  taskManager->Connect(
      controllersL.GetName(),  "TrajectoryResetInterface",
      trajectoriesL.GetName(), "TrajectoryResetInterface");

  taskManager->Connect(
      controllersR.GetName(),  "TrajectoryResetInterface",
      trajectoriesR.GetName(), "TrajectoryResetInterface");
      */

  taskManager->Connect(
      controllersL.GetName(),  devController::Output,
      wamL.GetName(),          devManipulator::Input );

  taskManager->Connect(
      controllersR.GetName(),  devController::Output,
      wamR.GetName(),          devManipulator::Input );

  taskManager->Connect(
      controllersL.GetName(),  devController::Feedback,
			wamL.GetName(),          devManipulator::Output );

  taskManager->Connect(
      controllersR.GetName(),  devController::Feedback,
			wamR.GetName(),          devManipulator::Output );

  taskManager->Connect(
      trajectoriesL.GetName(),   "WAM_Feedback",
			wamL.GetName(),            devManipulator::Output );

  taskManager->Connect(
      trajectoriesR.GetName(),   "WAM_Feedback",
			wamR.GetName(),            devManipulator::Output );

  taskManager->Connect(
      "ROS_PERIODIC_ADAPTER_L",  "RequiredAdapter",
			wamL.GetName(),           devManipulator::Output );

  taskManager->Connect(
      "ROS_PERIODIC_ADAPTER_R",  "RequiredAdapter",
			wamR.GetName(),           devManipulator::Output );

  taskManager->Connect(
      "ROS_CONTROLLER_DATA_PUB_ADAPTER_L",  "ProvidedAdapter",
      controllersL.GetName(),   "ControllerDataPublisher" );

  taskManager->Connect(
      "ROS_CONTROLLER_DATA_PUB_ADAPTER_R",  "ProvidedAdapter",
      controllersR.GetName(),   "ControllerDataPublisher" );

  taskManager->Connect(
      "ROS_TRAJECTORY_PUB_ADAPTER_L", "ProvidedAdapter",
      trajectoriesL.GetName(),       "TrajectoryPublisher" );

  taskManager->Connect(
      "ROS_TRAJECTORY_PUB_ADAPTER_R", "ProvidedAdapter",
      trajectoriesR.GetName(),       "TrajectoryPublisher" );

  taskManager->CreateAll();
  taskManager->StartAll();

  ros::spin();

  return 0;
}
