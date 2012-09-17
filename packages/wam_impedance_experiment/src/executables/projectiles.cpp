/***************************************************************************
 * This node simulates projectiles for the trajectory tracking task.  It
 * gives each projectile a mass and a velocity and simulates collisions
 * with the prosthesis.  
 *
 * To do this, it receives position information from the virtual prosthesis
 * or actual robot.  This node checks for collisions and uses conservation
 * of momentum to determine the resulting force.  Then it broadcasts the
 * resulting force, so that the virtual prosthesis simulation or the robot
 * controller can apply the simulated force.
 *
 * Projectiles are randomly generated as described below.  Projectile
 * generation is activated and deactivated via service calls.
 **************************************************************************/


#include "ros/ros.h"
#include <vector>
#include "wam_impedance_experiment/ProjectileInfo.h"
#include "wam_impedance_experiment/ControllerData.h"
#include "std_msgs/Empty.h"
#include "std_msgs/Float32.h"


// for virtual prosthesis position/velocity or wam position/velocity
double x_a = 0.0;
double x_a_old = 0.0;
double xd_a = 0.0;

// for graphics
double user_radius = 0.07;

// projectile parameters
int nProjectiles = 0; // tracks number of moving projectiles
int ncol = 0; // tracks number of collisions
int pnum = 0;
std::vector<float,std::allocator<float> > x_p (5,-1.9);
std::vector<float,std::allocator<float> > xd_p (5,0.0);
double speed = 0.5;
double m_p = 3.0; // projectile mass
double m = 3.0; // prosthesis mass
bool moving[5] = {false, false, false, false, false}; // keeps track of which projectiles are moving

// collision forces (use impulse = F*dt instead of force because timesteps vary for different nodes)
double impulse = 0.0;

// flag for generating projectiles or not
bool active = false;


// handle incoming prosthesis state data - get prosthesis position and mass, calculate velocity
void prosthesisInfoCallback(const wam_impedance_experiment::ControllerData::ConstPtr& msg) {
  if ((msg->name == "robot0") || (msg->name == "left")) {
    x_a = msg->pose.data[7];
    m = msg->m_y;
    xd_a = (x_a - x_a_old)/msg->deltat.toSec();
    x_a_old = x_a;
  }
}


void Quit(const std_msgs::Empty::ConstPtr& msg) {
  ROS_INFO("Projectiles: Quit message received.");
  ros::shutdown();
}


void toggleProjectiles(const std_msgs::Empty::ConstPtr& msg) {

  if (active) {
    
    active = false;
    for (int i=0;i<5;i++) {
      x_p[i] = -1.9;
      xd_p[i] = 0.0;
    }
    impulse = 0.0;

  } else {

    active = true;
    nProjectiles = 0;
    ncol = 0;
    for (int i=0;i<5;i++) {
      moving[i] = false;
    }

  }

}

int main(int argc, char **argv) {

	// seed random number generator
	srand(time(NULL));

  // set up ROS stuff
  ros::init(argc, argv, "projectiles");
  ros::NodeHandle n;
  ros::Rate loop_rate(1000); // publishing rate in Hz
  ros::Time t;
  ros::Time told;
  ros::Duration deltat;

  // subscriber for receiving quit command
  ros::Subscriber sub_quit = n.subscribe<std_msgs::Empty>("quit", 1000, Quit);

  // subscriber to get prosthesis position and mass
  ros::Subscriber sub_gains = n.subscribe<wam_impedance_experiment::ControllerData>("controller_data", 1000,prosthesisInfoCallback);

  // publisher for sending messages about projectile positions and collision forces
  ros::Publisher projectile_pub = n.advertise<wam_impedance_experiment::ProjectileInfo>("projectiles", 1000);
  wam_impedance_experiment::ProjectileInfo msg_projectiles;

  // subscriber for starting and stopping projectile generation
  ros::Subscriber sub_toggle_projectiles = n.subscribe<std_msgs::Empty>("toggle_projectiles", 1000, toggleProjectiles);

  t = ros::Time::now();

  // get user radius from parameter server
  if (n.getParam("/userRadius", user_radius)) {
    ROS_INFO("Projectiles: Got user radius for graphics: %f",user_radius);
  } else {
    ROS_WARN("Projectiles: Failed to get user radius for graphics. Using default value: %f",user_radius);
  }

  while (ros::ok()) {

    told = t;
    t = ros::Time::now();
    deltat = t - told;

    if (active) {

      // check for collisions and update projectile positions
      for (int i=0;i<5;i++) {
        if (moving[i]) {
          // update position
          x_p[i] += xd_p[i]*deltat.toSec();
          // check for collision
          if (fabs(x_a - x_p[i]) < user_radius + 0.0015) {
            // update number of collisions - keep a count because if more than one happens at a time, they cancel out.
            if (xd_p[i] > 0) {
              ncol++;
            } else if (xd_p[i] < 0) {
              ncol--;
            }
            xd_p[i] = 0;
            moving[i] = false;
            nProjectiles--;
          }
        } else {
          x_p[i] = -1.9;
        }
      }

      // if collision, calculate resulting force
      if (ncol != 0) {
        // Calculate velocity that results from collision
        // We don't care about the projectile velocity, because it just disappears
        //xd_p[i] = 2*m_p/(m+m_p)*xd_a + (m_p-m)/(m_p+m)*xd_p[i];
        // notes: if multiple projectiles hit from the same direction, multiply the mass by the number
        //        of projectiles, fabs(ncol). to get the direction, use the sign,
        //        ncol/fabs(ncol).
        double temp = (m-fabs(ncol)*m_p)/(m+fabs(ncol)*m_p)*xd_a + 2.0*fabs(ncol)*m_p/(m+fabs(ncol)*m_p)*ncol/fabs(ncol)*speed;
        // Calculate impulse = change in momentum on user with F*deltat = m*deltav.
        // Pass impulse instead of force because timesteps are different for different nodes.
        impulse = m*(temp - xd_a);
      } else {
        impulse = 0.0;
      }

      ncol = 0;

      // perhaps randomly trigger a perturbation
      if ((nProjectiles < 5) && ((double)rand()/(double)RAND_MAX < 0.001)) {
        // choose an inactive perturbation (this is not always the next one in line, because they might collide out of order)
        for (pnum=0;pnum<5;pnum++) {
          //pnum = (pnum+1)%5;
          if (!moving[pnum]) {
            break;
          }
        }
        nProjectiles++;
        moving[pnum] = true;
        // choose a direction and start the projectile's motion
        if (rand()%2 == 0) {
          if (2.0>(x_a + 0.5)) {
            x_p[pnum] = 1.8;
          } else {
            x_p[pnum] = (x_a + 0.5);
          }
          xd_p[pnum] = -1.0*speed;
        } else {
          if (-2.0<(x_a - 0.5)) {
            x_p[pnum] = -1.8;
          } else {
            x_p[pnum] = (x_a - 0.5);
          }
          xd_p[pnum] = speed;
        }
      }



    }
    
    // broadcast projectile positions and collision forces
    msg_projectiles.t = t;
    msg_projectiles.impulse = impulse;
    msg_projectiles.x_p = x_p;
    projectile_pub.publish(msg_projectiles);


    ros::spinOnce();
    loop_rate.sleep();
  }


  return 0;

}
