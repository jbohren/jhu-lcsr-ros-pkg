/****************************************************************************************
 *
 * This node creates the graphics used for the wam impedance experiment and supplies
 * the keyboard interface. The keyboard interface is designed to be used with the
 * experiment manager node, because the result of a key press will depend on the
 * experiment state. Thus, the keyboard interface simply publishes the number of the
 * key press. The only exception is when a quit key is pressed; in this case, the key
 * press is published along with a quit message, and this node quits.
 *
 ***************************************************************************************/

#include <vector>
#include "ros/ros.h"
#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/glut.h>
#include <stdio.h>
#include "wam_impedance_experiment/key_defs.h"
#include "wam_impedance_experiment/GraphicsInfo.h"
#include "wam_impedance_experiment/ChangeState.h"
#include "wam_impedance_experiment/TaskType.h"
#include "wam_impedance_experiment/SetType.h"
#include "wam_impedance_experiment/States.h"
#include "std_msgs/Empty.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Float64.h"
#include "wam_impedance_experiment/Bool.h"
#include "wam_impedance_experiment/ProjectileInfo.h"
#include "wam_impedance_experiment/TrajectoryInfo.h"
#include "wam_impedance_experiment/DAQinput.h"
#include "wam_impedance_experiment/ControllerData.h"


// OpenGL and GLUT stuff in this code originally based on the tutorials at www.cs.uoi.gr/~fudos/opengl-tutorial/search.htm
// ROS stuff based on ROS beginner tutorials

ros::NodeHandle* n;
ros::Publisher* quit_pub;
ros::Publisher* key_press_pub;
ros::Publisher* tracking_goal_pub;

// some useful constants
const double PI = 3.14159265;
const int WIN_WIDTH = 848;
const int WIN_HEIGHT = 480;
const int WIN_POS_X = 102;
const int WIN_POS_Y = 77;
const void *FONT = GLUT_BITMAP_HELVETICA_12;

// for walls:
// IMPORTANT: these limits are hard-coded both here and in the controller. if you change one, you must go find and change the other.
const double limitL_controller = -0.2;
const double limitR_controller = 0.2;
double dropoff_controller = 0.05;

double graphics_height = 0.82; // graphics y position offset for users of different heights
double position_scale = 14.0; // scale factor to make graphics line up with wam movement
double user_radius = 0.07;

// initialize variables to hold experiment state info
stateT state = setup1;
taskType task = forcemin;
double x_a1 = 0.0;
double x_a2 = 0.0;
bool maxCommandSpeed = false;
double x_d1 = 0.0;
double x_d2 = 0.0;
double Fenv = 0.0;
int direction = 0;
//int nProjectiles = 0;
std::vector<float,std::allocator<float> > x_p (5,-2.6); // initialize all projectiles to be off screen

// intialize variables for info needed to print text
setType set_type = training;
int nTrials = 10;
int trial_number = 1;
int trial_length = 15;

// initialize variables to hold other graphics info
double phase = 0.0;
double limitL = -3.0;
double limitR = 3.0;

// initialize mode variables
bool debug_mode = false; // in debug mode, desired position(s) are displayed


// sine wave parameters
const double sinescale = 0.09;
const double amp1 = 0.8*sinescale;
const double amp2 = -0.6*sinescale;
const double freq1 = 1.2;
const double freq2 = 1.9;

int subjectNumber = -1;

GLvoid initGL()
{
	glClearColor(0,0,0,1); // Set background color to black
}


void init_scene()
{
}


GLvoid window_reshape(GLsizei width, GLsizei height)
{
	glViewport(0, 0, width, height); // defines location and size of viewport
	// fix the width-height ratio
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(45, (GLdouble)width/(GLdouble)height, 1, 10);
	glMatrixMode(GL_MODELVIEW);
}


// Function to print strings from glutfonts.c, Copyright (c) Gerard Lanois, 1998.
void print_bitmap_string(void* font, char* s)
{
   if (s && strlen(s)) {
      while (*s) {
         glutBitmapCharacter(font, *s);
         s++;
      }
   }
}


void render_scene()
{
  // display graphics or instructions to the user, depending on state

  char mystring[500];

  // set text color to white
  glColor3f(1,1,1);

  // text starting position
  double x = -1.0;
  double y = 0.5; 

  // line height
  double dy = 0.2;

  GLUquadricObj *q;

  switch (state) {

    case graphicsCalibration1:
    case graphicsCalibration3:

      // just show actual position and some instructions text.

      y = -1.8; 
      x = -2.0;
      glColor3f(1,1,1); 

      if (state == graphicsCalibration1) {
        ROS_DEBUG("display graphicsCalibration1 message");
        sprintf(mystring,"Calibrate graphics height using +/- keys. Then advance state.");
      } else {
        ROS_DEBUG("display graphicsCalibration3 message");
        sprintf(mystring,"Calibrate graphics scale using [ and ] keys. Then advance state.");
      }

      glRasterPos2f(x,y);
      print_bitmap_string(GLUT_BITMAP_HELVETICA_18,mystring);

      glPushMatrix();
      glTranslatef(0,graphics_height,0);
      glPushMatrix();
      glScalef(position_scale, position_scale, 1);

      // need to flip the x axis to match up with wam axes.
      // do this by rotating 180 degrees about the y axis.
      glPushMatrix();
      glRotatef(180,0,1,0);

      q = gluNewQuadric();

      glColor3f(.9,.2,.2); 
      glPushMatrix();
      glTranslatef(x_a1,0,0);
      gluDisk(q, 0, user_radius, 30, 1);
      glPopMatrix();

      gluDeleteQuadric(q);

      glPopMatrix();
      glPopMatrix();
      
      break;

    case startingTrial2:
    case runningTrial:

      glTranslatef(0,graphics_height,0);
      glPushMatrix();
      glScalef(position_scale, position_scale, 1);

      q = gluNewQuadric();

      // need to flip the x axis to match up with wam axes.
      // do this by rotating 180 degrees about the y axis.
      glPushMatrix();
      glRotatef(180,0,1,0);

      // draw walls at workspace limits implemented in SE3 controller.
      // draw walls at limit plus ball radius
      if (x_a1 < limitL_controller) {
        glColor3f(0.3,0.3,0.3);
      } else {
        glColor3f(0.6,0.6,0.6);
      }
      glRectf((limitL_controller-user_radius),-0.6/position_scale,(limitL_controller-user_radius)-0.4/position_scale,0.6/position_scale);
      if (x_a1 > limitR_controller) {
        glColor3f(0.3,0.3,0.3);
      } else {
        glColor3f(0.6,0.6,0.6);
      }
      glRectf((limitR_controller+user_radius),-0.6/position_scale,(limitR_controller+user_radius)+0.4/position_scale,0.6/position_scale);

      // display user position
      // draw ball, if on screen. otherwise, draw an indicator arrow.
      // Set current color - yellow at max speed, red otherwise.
      if (debug_mode) {
        if (maxCommandSpeed) {
          glColor3f(.9,.8,.2); 
        } else {
          glColor3f(.9,.2,.2); 
        }
        if (fabs(x_a1) < 3.5) {
          glPushMatrix();
          glTranslatef(x_a1,0,0);
          gluDisk(q, 0, user_radius, 30, 1);
          glPopMatrix();
        } else if (x_a1 >= 3.5) {
          glLineWidth(3);
          glBegin(GL_LINES);
          glVertex2f(2.9,0);
          glVertex2f(3.1,0);
          glVertex2f(3.1,0);
          glVertex2f(3.07,0.03);
          glVertex2f(3.1,0);
          glVertex2f(3.07,-0.03);
          /*
             glVertex2f(0,1.7);
             glVertex2f(0,1.9);
             glVertex2f(0,1.9);
             glVertex2f(0.03,1.87);
             glVertex2f(0,1.9);
             glVertex2f(-0.03,1.87);
             */
          glEnd();
          glLineWidth(1);
        } else {
          glLineWidth(3);
          glBegin(GL_LINES);
          glVertex2f(-2.9,0);
          glVertex2f(-3.1,0);
          glVertex2f(-3.1,0);
          glVertex2f(-3.07,0.03);
          glVertex2f(-3.1,0);
          glVertex2f(-3.07,-0.03);
          /*
             glVertex2f(0,-1.7);
             glVertex2f(0,-1.9);
             glVertex2f(0,-1.9);
             glVertex2f(0.03,-1.87);
             glVertex2f(0,-1.9);
             glVertex2f(-0.03,-1.87);
             */
          glEnd();
          glLineWidth(1);
        }
      }

      // draw max command speed indicator
      if (maxCommandSpeed) {
        glColor3f(.7,.6,.2); 
      } else {
        glColor3f(0.6,0.6,0.6); // Set current color
      }
      glRectf(-0.3/position_scale,-2.0/position_scale,0.3/position_scale,-2.4/position_scale);


      // if trajectory tracking task, display trajectory and projectiles
      if (task == tracking) {
        // draw trajectory
        glColor3f(1.0,1.0,1.0); // Set current color
        glLineWidth(2);
        glBegin(GL_LINES);
        for (int j=-520; j<520; j++) {
          if ((j >= 200 * limitL) && (j <= 200 * limitR)) {
            double y1 = direction*(amp1*sin(freq1*((double)j/200.0 + phase)) + amp2*sin(freq2*((double)j/200.0 + phase)));
            double y2 = direction*(amp1*sin(freq1*((double)(j+1)/200.0 + phase)) + amp2*sin(freq2*((double)(j+1)/200.0 + phase)));
            glVertex2f(y1,(double)j/200.0/position_scale);
            glVertex2f(y2,(double)(j+1)/200.0/position_scale);
          }
        }
        glEnd();
        glLineWidth(1);

        // draw projectiles
        glColor3f(1,0.6,0); // Set current color
        for (int i=0;i<5;i++) {
          glPushMatrix();
          glTranslatef(x_p[i],0,0);
          gluDisk(q, 0, 0.003, 30, 1);
          glPopMatrix();
        }
      }

      // if debug mode, also display user desired position and moving object desired and actual positions
      if (debug_mode) {
        // in force minimization task, draw moving object desired and actual positions
        if (task == forcemin) {
          // desired
          glColor3f(0.2,0.9,0.2); // Set current color
          glPushMatrix();
          glTranslatef(x_d2,0,0);
          gluDisk(q, 0, 0.006, 30, 1);
          glPopMatrix();
          //actual
          glColor3f(1.0,0.4,0.7); // Set current color
          glPushMatrix();
          glTranslatef(x_a2,0,0);
          gluDisk(q, 0, 0.005, 30, 1);
          glPopMatrix();
        }
        // draw user desired position
        glColor3f(0,0,1); // Set current color
        glPushMatrix();
        glTranslatef(x_d1,0,0);
        gluDisk(q, 0, 0.004, 30, 1);
        glPopMatrix();
      }

      // in forcemin task, show force vector
      if (task == forcemin) {
        double sign = (Fenv > 0) - (Fenv < 0); // sign of force vector
        const double force_scale = 0.05;
        if (sign != 0) {
          glLineWidth(2);
          glColor3f(1,1,1);
          glBegin(GL_LINES);
          glVertex2f(x_a1,0.0);
          glVertex2f(x_a1,-0.9/position_scale);
          glVertex2f(x_a1,-0.9/position_scale);
          glVertex2f(x_a1 + Fenv*force_scale/position_scale,-0.9/position_scale);
          glVertex2f(x_a1 + Fenv*force_scale/position_scale,-0.9/position_scale);
          glVertex2f(x_a1 - sign*0.05/position_scale + Fenv*force_scale/position_scale,0.04/position_scale - 0.9/position_scale);
          glVertex2f(x_a1 + Fenv*force_scale/position_scale,-0.9/position_scale);
          glVertex2f(x_a1 - sign*0.05/position_scale + Fenv*force_scale/position_scale,-0.04/position_scale - 0.9/position_scale);
          /*
          if (fabs(Fenv*force_scale) > 0.1) {
            // regular size arrow
            glVertex2f(x_a1 + Fenv*force_scale,0);
            glVertex2f(x_a1 - sign*0.05 + Fenv*force_scale,0.04);
            glVertex2f(x_a1 + Fenv*force_scale,0);
            glVertex2f(x_a1 - sign*0.05 + Fenv*force_scale,-0.04);
          } else if (fabs(Fenv*force_scale) > 0.05) {
            // scale down arrow size
            glVertex2f(x_a1 + Fenv*force_scale,0);
            glVertex2f(x_a1 + sign*0.05,0.04*pow((fabs(Fenv*force_scale)-0.05)/0.05,2));
            glVertex2f(x_a1 + Fenv*force_scale,0);
            glVertex2f(x_a1 + sign*0.05,-0.04*pow((fabs(Fenv*force_scale)-0.05)/0.05,2));
          } else {
            // now it's just the line
          }
          */
          glEnd();
        }
      }

      glPopMatrix();
      glPopMatrix();
      gluDeleteQuadric(q);
      break;

    case setup1:
      ROS_DEBUG("display setup1 message");
      y = -1.8; 
      x = -1.55;
      sprintf(mystring,"Subject %d. Activate motors and advance state.",subjectNumber);
      glRasterPos2f(x,y);
      print_bitmap_string(GLUT_BITMAP_HELVETICA_18,mystring);
      y -= 0.1;
      break;

    case setup2:
    case setupTracking2:
    case setupForcemin2:
    case setupForcemin4:
    case cleanup1:
    case cleanup3:
      ROS_DEBUG("display controller transition message");
      y = -1.8; 
      x = -1.1;
      sprintf(mystring,"Waiting for controller transition...");
      glRasterPos2f(x,y);
      print_bitmap_string(GLUT_BITMAP_HELVETICA_18,mystring);
      y -= 0.1;
      break;

    case setup3:
      ROS_DEBUG("display setup3 message");
      y = -1.8; 
      x = -2.2;
      if (task == tracking) {
        sprintf(mystring,"Prepare for tracking task. Remove connecting bar, then advance state.");
      } else if (task == forcemin) {
        sprintf(mystring,"Prepare for forcemin task. Unstrap right arm, then advance state.");
      }
      glRasterPos2f(x,y);
      print_bitmap_string(GLUT_BITMAP_HELVETICA_18,mystring);
      break;

    case setupTracking1:
    case setupForcemin1:
    case graphicsCalibration2:
    case graphicsCalibration4:
      ROS_DEBUG("display setpoint transition message");
      y = -1.8; 
      x = -0.9;
      sprintf(mystring,"Waiting to reach setpoint...");
      glRasterPos2f(x,y);
      print_bitmap_string(GLUT_BITMAP_HELVETICA_18,mystring);
      break;

    case setupForcemin3:
      ROS_DEBUG("display setupForcemin3 message");
      y = -1.8; 
      x = -2.0;
      sprintf(mystring,"Attach connecting bar (right, then left). Then advance state.");
      glRasterPos2f(x,y);
      print_bitmap_string(GLUT_BITMAP_HELVETICA_18,mystring);
      break;

    case waitingForUser:
      ROS_DEBUG("display waitingForUser messsage");
      // display number of trials completed
      y = -1.8;
      x = -1.6; 
      if (set_type == practice) {
        sprintf(mystring,"Practice trial %i of %i. Press enter to begin.",trial_number,nTrials);
      } else if (set_type == experiment) {
        sprintf(mystring,"Experiment trial %i of %i. Press enter to begin.",trial_number,nTrials);
      } else {
        sprintf(mystring,"ERROR: Unknown set type.");
      }
      glRasterPos2f(x,y);
      print_bitmap_string(GLUT_BITMAP_HELVETICA_18,mystring);
      break;

    case timedBreak:
      ROS_DEBUG("display betweenSets message");
      // display time remaining on break
      y = -1.8;
      x = -1.6; 
      sprintf(mystring,"Set complete. Please notify the experimenter.");
      glRasterPos2f(x,y);
      print_bitmap_string(GLUT_BITMAP_HELVETICA_18,mystring);
      break;

    case instructions:
      ROS_DEBUG("display task instructions");
      // give user instructions about the task
      if (task == forcemin) {
        y = -1.8+0.5*dy; 
        x = -2.75;
        sprintf(mystring,"In this task, the goal is to control the robot to minimize the length of the attached");
        glRasterPos2f(x,y);
        print_bitmap_string(GLUT_BITMAP_HELVETICA_18,mystring);
        y -= dy;
        sprintf(mystring,"arrow by moving in the direction of the arrow. Press enter to continue.");
        glRasterPos2f(x,y);
        print_bitmap_string(GLUT_BITMAP_HELVETICA_18,mystring);
      } else if (task == tracking) {
        y = -1.8+0.5*dy; 
        x = -2.15;
        sprintf(mystring,"In this task, the goal is to control the robot to follow the curve");
        glRasterPos2f(x,y);
        print_bitmap_string(GLUT_BITMAP_HELVETICA_18,mystring);
        y -= dy;
        sprintf(mystring,"on the screen as closely as possible. Press enter to continue.");
        glRasterPos2f(x,y);
        print_bitmap_string(GLUT_BITMAP_HELVETICA_18,mystring);
      } else {
        y = -1.8; 
        x = -1.8;
        sprintf(mystring,"ERROR: Unknown task type. Press enter to continue.");
        glRasterPos2f(x,y);
        print_bitmap_string(GLUT_BITMAP_HELVETICA_18,mystring);
      }
      /*
      y -= 1.5*dy;
      if (set_type == practice) {
        sprintf(mystring,"This set will be a practice set, consisting of %i trials of %d seconds each.", nTrials, trial_length);
      } else if (set_type == experiment) {
        sprintf(mystring,"This set will be an experiment set, consisting of %i trials of %d seconds each.", nTrials, trial_length);
      } else {
        sprintf(mystring,"ERROR: Unknown set type.");
      }
      glRasterPos2f(x,y);
      print_bitmap_string(GLUT_BITMAP_HELVETICA_18,mystring);
      */
      break;

    case startingTrial1:
      ROS_DEBUG("display startingTrial1 message");
      y = -1.8; 
      x = -1.0;
      sprintf(mystring,"Beginning trial... Please wait.");
      glRasterPos2f(x,y);
      print_bitmap_string(GLUT_BITMAP_HELVETICA_18,mystring);
      break;

    case writingDataFile:
      ROS_DEBUG("display writingDataFile message");
      y = -1.8; 
      x = -1.1;
      sprintf(mystring,"Writing data file... Please wait.");
      glRasterPos2f(x,y);
      print_bitmap_string(GLUT_BITMAP_HELVETICA_18,mystring);
      break;

    case resetWam1:
    case resetWam2:
    case resetWam3:
      ROS_DEBUG("display resetWam message");
      y = -1.8; 
      x = -0.8;
      sprintf(mystring,"Resetting... Please wait.");
      glRasterPos2f(x,y);
      print_bitmap_string(GLUT_BITMAP_HELVETICA_18,mystring);
      break;

    case rating:
      y = -1.8;
      x = -2.3; 
      ROS_DEBUG("display rating instructions");
      // give user instructions on selecting a rating (1-10)
      sprintf(mystring,"Please rate your performance in this trial from 1 (Worst) to 5 (Best).");
      glRasterPos2f(x,y);
      print_bitmap_string(GLUT_BITMAP_HELVETICA_18,mystring);
      break;

    case experimentDone:
      y = -1.8;
      x = -1.1; 
      ROS_DEBUG("display done instructions");
      // display done message
      sprintf(mystring,"Done. Please notify experimenter.");
      glRasterPos2f(x,y);
      print_bitmap_string(GLUT_BITMAP_HELVETICA_18,mystring);
      y -= 0.1;
      break;

    case cleanup2:
      ROS_DEBUG("display cleanup2 message");
      y = -1.8;
      x = -1.5; 
      sprintf(mystring,"Remove connecting bar, then advance state.");
      glRasterPos2f(x,y);
      print_bitmap_string(GLUT_BITMAP_HELVETICA_18,mystring);
      break;

    case cleanup4:
      ROS_DEBUG("display cleanup4 message");
      y = -1.8;
      x = -2.3; 
      sprintf(mystring,"Activating gravity compensation. Re-holster WAMs before quitting.");
      glRasterPos2f(x,y);
      print_bitmap_string(GLUT_BITMAP_HELVETICA_18,mystring);
      break;

    default:
      // display nothing
      break;
  }
}


GLvoid window_display()
{
	// This function redraws the scene
	glClear(GL_COLOR_BUFFER_BIT); // Clear the color buffer
	glLoadIdentity(); // reset the modelview transformation
	gluLookAt(0,0,5,0,0,0,0,1,0); // Define the center of projection, view reference point, and view up vector
	render_scene(); // renders the scene (user-defined)
	glutSwapBuffers();
}


void quit()
{
  ros::shutdown();
  exit(0);
}


GLvoid window_key(unsigned char key, int x, int y)
{
  // set key press flag and store key number
  std_msgs::Int32 key_msg;
  key_msg.data = key;
  key_press_pub->publish(key_msg);
  ROS_INFO("Published key press %d.",key);
    
  // process quit keys
  if (debug_mode && (key == KEY_ESC)) {
  //if (key == KEY_ESC) {
    // publish quit message for all nodes
    ROS_DEBUG("Experiment quit key received.");
    std_msgs::Empty quit_msg;
    quit_pub->publish(quit_msg);
    quit();
  } else if (key == KEY_SHIFT_Q) {
    // just quit graphics node
    ROS_DEBUG("Graphics quit key received.");
    quit();
  } else if (((state == graphicsCalibration1) || debug_mode) && ((key == KEY_PLUS) || (key == KEY_EQUAL))) {
    // shift graphics up
    if (graphics_height < 1.2) {
      graphics_height += 0.02;
    }
    ROS_INFO("Graphics: height = %f.",graphics_height);
  } else if (((state == graphicsCalibration1) || debug_mode) && ((key == KEY_MINUS) || (key == KEY_US))) {
    // shift graphics down
    if (graphics_height > -1.2) {
      graphics_height -= 0.02;
    }
    ROS_INFO("Graphics: height = %f.",graphics_height);
  } else if (((state == graphicsCalibration3) || debug_mode) && ((key == KEY_LBRACK) || (key == KEY_LCBRACK))) {
    // shrink graphics
    position_scale -= 0.1;
    ROS_INFO("Graphics: position_scale = %f.",position_scale);
  } else if (((state == graphicsCalibration3) || debug_mode) && ((key == KEY_RBRACK) || (key == KEY_RCBRACK))) {
    // stretch graphics
    position_scale += 0.1;
    ROS_INFO("Graphics: position_scale = %f.",position_scale);
  } else if (key == KEY_G) {
    // print out graphics calibration
    ROS_INFO("Graphics: height = %f.",graphics_height);
    ROS_INFO("Graphics: position_scale = %f.",position_scale);
  }
}


GLvoid window_idle()
{
}


void update(int value)
{
  ros::Time t = ros::Time::now();
  ROS_DEBUG("time %f",t.toSec());

	glutPostRedisplay(); // tells GLUT to redraw the window asap
	glutTimerFunc(10,update,0);
  ros::spinOnce();
}

void graphicsInfoCallback(const wam_impedance_experiment::GraphicsInfo::ConstPtr& msg) {
  /*
  if ((ros::Time::now() - msg->t).toSec() > 0.1) {
    ROS_WARN("Graphics info message delay: %f ms (nominal period 10 ms)",(ros::Time::now() - msg->t).toSec()*1000);
  }
  */
  task = (taskType)(msg->task);
  direction = msg->direction;
  limitL = msg->limitL;
  limitR = msg->limitR;
  phase = msg->phase;

  // now publish the goal position so it can be logged
  std_msgs::Float64 tracking_goal_msg;
  tracking_goal_msg.data = direction*(amp1*sin(freq1*phase) + amp2*sin(freq2*phase));
  tracking_goal_pub->publish(tracking_goal_msg);

}

void projectilesCallback(const wam_impedance_experiment::ProjectileInfo::ConstPtr& msg) {
  /*
  if ((ros::Time::now() - msg->t).toSec() > 0.03) {
    ROS_WARN("Projectiles message delay: %f ms (nominal period 1 ms)",(ros::Time::now() - msg->t).toSec()*1000);
  }
  */
  x_p = msg->x_p;
  for (unsigned int i=0; i<x_p.size(); i++) {
    x_p[i] = x_p[i];
  }
}

void actualPositionCallback(const wam_impedance_experiment::ControllerData::ConstPtr& msg) {
  // the message contains a whole bunch of information from the controller, including the
  // current SE3 pose of a robot. there may be messages coming from multiple robots, in 
  // which case they will be marked with names. check the name to see which ones to keep.
  if ((msg->name.compare("robot0") == 0) || (msg->name.compare("left") == 0)) {
    // note that the pose is stored in a Float64MultiArray, which is a vector containing the matrix elements.
    // so the endpoint y-position, which is the [1][3] element (2nd row, 4th col), is element [7].
    x_a1 = (msg->pose.data[7]);
  } else if ((msg->name.compare("robot1") == 0) || (msg->name.compare("right") == 0)) {
    // note that the pose is stored in a Float64MultiArray, which is a vector containing the matrix elements.
    // so the endpoint y-position, which is the [1][3] element (2nd row, 4th col), is element [7].
    x_a2 = (msg->pose.data[7]);
  }
}

void trajectoryCallback(const wam_impedance_experiment::TrajectoryInfo::ConstPtr& msg) {
  // the message contains a whole bunch of trajectory information, which includes the desired
  // SE3 pose of the robot. there may be messages coming from multiple robots, in which case
  // they will be marked with names.
  if ((msg->name.compare("robot0") == 0) || (msg->name.compare("left") == 0)) {
    // note that the pose is stored in a Float64MultiArray, which is a vector containing the matrix elements.
    // so the endpoint y-position, which is the [1][3] element (2nd row, 4th col), is element [7].
    x_d1 = (msg->setpointSE3.data[7]);
    maxCommandSpeed = msg->maxCommandSpeed;
  } else if ((msg->name.compare("robot1") == 0) || (msg->name.compare("right") == 0)) {
    // note that the pose is stored in a Float64MultiArray, which is a vector containing the matrix elements.
    // so the endpoint y-position, which is the [1][3] element (2nd row, 4th col), is element [7].
    x_d2 = (msg->setpointSE3.data[7]);
  }
}

void daqInputCallback(const wam_impedance_experiment::DAQinput::ConstPtr& msg) {
  // this message contains all the info read by the DAQ. the environment force comes from the second force sensor.
  Fenv = msg->force2;
}

bool setMode(wam_impedance_experiment::Bool::Request  &req,
             wam_impedance_experiment::Bool::Response &res )
{
  debug_mode = req.data;
  return true;
}


bool changeState(wam_impedance_experiment::ChangeState::Request  &req,
                 wam_impedance_experiment::ChangeState::Response &res )
{
  //ROS_INFO("State: %d", req.state);
  state = (stateT)(req.state);
  trial_number = req.trial_number;
  nTrials = req.nTrials;
  trial_length = req.trial_length;
  set_type = (setType)(req.set_type);
  task = (taskType)(req.task);
  return true;
}


int main(int argc, char **argv) {

  // set up ROS stuff
  ros::init(argc, argv, "graphics");
  n = new ros::NodeHandle;
  ros::Time t;
  ros::Time told;
  ros::Duration deltat;

  // subscribers to get info for graphics
  // graphics info - for drawing sine wave
  ros::Subscriber sub_graphics_info = n->subscribe<wam_impedance_experiment::GraphicsInfo>("graphics_info", 1000,graphicsInfoCallback);
  // projectile positions
  ros::Subscriber sub_projectiles = n->subscribe<wam_impedance_experiment::ProjectileInfo>("projectiles", 1000,projectilesCallback);
  // daq input - to get environment force
  ros::Subscriber sub_daq_input = n->subscribe<wam_impedance_experiment::DAQinput>("daq_input", 1000,daqInputCallback);
  // trajectory - to get desired positions for both arms
  ros::Subscriber sub_trajectory = n->subscribe<wam_impedance_experiment::TrajectoryInfo>("trajectory", 1000,trajectoryCallback);
  // subscriber to get actual position
  ros::Subscriber sub_endpoint_pose = n->subscribe<wam_impedance_experiment::ControllerData>("controller_data", 1000,actualPositionCallback);

  // publishers for quit key press and general key press
  quit_pub = new ros::Publisher;
  *quit_pub = n->advertise<std_msgs::Empty>("quit", 1000);
  key_press_pub = new ros::Publisher;
  *key_press_pub = n->advertise<std_msgs::Int32>("key_press",1000);

  // publisher for tracking goal
  tracking_goal_pub = new ros::Publisher;
  *tracking_goal_pub = n->advertise<std_msgs::Float64>("tracking_goal",1000);

  // advertise services for things that don't happen often
  ros::ServiceServer server_set_graphics_mode = n->advertiseService("set_graphics_mode", setMode);
  ros::ServiceServer server_change_state = n->advertiseService("change_state", changeState);

  // get subject number
  if (n->getParam("/subjectNumber", subjectNumber)) {
    if (subjectNumber == 0) {
      ROS_WARN("Graphics: Using default subject number 0. To specify a different subject number, use \"n:=##\" with the roslaunch command.");
    } else {
      ROS_INFO("Graphics: Running experiment for subject number %d.",subjectNumber);
    }
  } else {
    ROS_WARN("Graphics: Unable to retrieve subject number from parameter server. Default to subject number 0.");
    subjectNumber = 0;
  }

  // get user radius from parameter server
  if (n->getParam("/userRadius", user_radius)) {
    ROS_INFO("Graphics: Got user radius: %f",user_radius);
  } else {
    ROS_WARN("Graphics: Failed to get user radius. Using default value: %f",user_radius);
  }

  if (ros::ok()) {

    //Set up glut
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE); // Color display mode, RGB, and A = alpha, which defines opacity; also activate double buffering
    glutInitWindowSize(WIN_WIDTH, WIN_HEIGHT); // Set window size
    glutInitWindowPosition(WIN_POS_X, WIN_POS_Y); // Window placement. (0,0) corresponds to top left of screen.
    glutCreateWindow("Experiment!"); // Creates the window and gives it a name
    //glutFullScreen();
    initGL(); // user-defined function to initialize OpenGL
    init_scene(); // user-defined function to prepare objects in the scene for rendering
    glutDisplayFunc(&window_display); // Tells the program what function to call when the window needs to be redrawn. window_display is user-defined
    glutReshapeFunc(&window_reshape); // Tells the program what function to call when the window is resized
    glutKeyboardFunc(&window_key); // Tells the program what function to call when the user presses a key
    glutIdleFunc(&window_idle); // Tells the program what function to call when nothing is happening
    t = ros::Time::now();
    glutTimerFunc(10, update, 0); // Calls the function "update" after 10 ms
    glutMainLoop(); // start the main loop!

  }

  delete quit_pub;
  delete key_press_pub;
  delete tracking_goal_pub;
  delete n;

  return 0;
}
