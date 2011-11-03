#include "ros/ros.h"
#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/glut.h>
#include <stdio.h>
#include "simulation_impedance_experiment/keyboard.h"
#include "simulation_impedance_experiment/ExperimentState.h"
#include "simulation_impedance_experiment/GraphicsInfo.h"
#include "simulation_impedance_experiment/SetGraphicsMode.h"
#include "simulation_impedance_experiment/ChangeState.h"
#include "simulation_impedance_experiment/TaskType.h"
#include "simulation_impedance_experiment/SetType.h"
#include "simulation_impedance_experiment/States.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Int32.h"


// OpenGL and GLUT stuff in this code originally based on the tutorials at www.cs.uoi.gr/~fudos/opengl-tutorial/search.htm
// ROS stuff based on ROS beginner tutorials

ros::NodeHandle* n;
ros::Publisher* quit_pub;
ros::Publisher* key_press_pub;

// some useful constants
const double PI = 3.14159265;
const int WIN_WIDTH = 480;
const int WIN_HEIGHT = 480;
const void *FONT = GLUT_BITMAP_HELVETICA_12;


// initialize variables to hold experiment state info
stateT state = instructions;
taskType task = forcemin;
double x_a = 0.0;
double x_d1 = 0.0;
double x_d2 = 0.0;
double Fenv = 0.0;
int direction = 0;
int nProjectiles = 0;
boost::array<double,5> x_p = {{-4.0,-4.0,-4.0,-4.0,-4.0}}; // initialize all projectiles to be off screen

// intialize variables for info needed to print text
setType set_type = training;
int nTrials = 10;
int trial_number = 1;
int trial_length = 15;
bool visual = false;
bool vibrotactile = false;
bool skin_stretch = false;

// initialize variables to hold other graphics info
double phase = 0.0;
double limitL = -2.0;
double limitR = 2.0;

// initialize mode variables
bool showforce = false; // flag for displaying force vector
bool debug_mode = false; // in debug mode, desired position(s) are displayed


// sine wave parameters
const double amp1 = 0.8;
const double amp2 = -0.6;
const double freq1 = 1.2;
const double freq2 = 1.9;

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

  switch (state) {

    case runningTrial:

      GLUquadricObj *q;
      q = gluNewQuadric();

      // display user position
      // draw ball, if on screen. otherwise, draw an indicator arrow.
      glColor3f(.9,.2,.2); // Set current color
      if (fabs(x_a) < 2.1) {
        glPushMatrix();
        glTranslatef(0,x_a,0);
        gluDisk(q, 0, 0.07, 30, 1);
        glPopMatrix();
      } else if (x_a >= 2.1) {
        glLineWidth(3);
        glBegin(GL_LINES);
        glVertex2f(0,1.7);
        glVertex2f(0,1.9);
        glVertex2f(0,1.9);
        glVertex2f(0.03,1.87);
        glVertex2f(0,1.9);
        glVertex2f(-0.03,1.87);
        glEnd();
        glLineWidth(1);
      } else {
        glLineWidth(3);
        glBegin(GL_LINES);
        glVertex2f(0,-1.7);
        glVertex2f(0,-1.9);
        glVertex2f(0,-1.9);
        glVertex2f(0.03,-1.87);
        glVertex2f(0,-1.9);
        glVertex2f(-0.03,-1.87);
        glEnd();
        glLineWidth(1);
      }

      // if trajectory tracking task, display trajectory and projectiles
      if (task == tracking) {
        // draw trajectory
        glColor3f(1.0,1.0,1.0); // Set current color
        glLineWidth(2);
        glBegin(GL_LINES);
        for (int j=-600; j<600; j++) {
          if ((j >= 200 * limitL) && (j <= 200 * limitR)) {
            double y1 = direction*(amp1*sin(freq1*((double)j/200.0 + phase)) + amp2*sin(freq2*((double)j/200.0 + phase)));
            double y2 = direction*(amp1*sin(freq1*((double)(j+1)/200.0 + phase)) + amp2*sin(freq2*((double)(j+1)/200.0 + phase)));
            glVertex2f((double)j/200.0,y1);
            glVertex2f((double)(j+1)/200.0,y2);
          }
        }
        glEnd();
        glLineWidth(1);

        // draw projectiles
        glColor3f(1,1,0); // Set current color
        for (int i=0;i<nProjectiles;i++) {
          glPushMatrix();
          glTranslatef(0,x_p[i],0);
          gluDisk(q, 0, 0.03, 30, 1);
          glPopMatrix();
        }
      }

      // if debug mode, also display user desired position and moving object desired position
      if (debug_mode) {
        // in force minimization task, draw moving object desired position
        if (task == forcemin) {
          glColor3f(0.2,0.9,0.2); // Set current color
          glPushMatrix();
          glTranslatef(0,x_d2,0);
          gluDisk(q, 0, 0.07, 30, 1);
          glPopMatrix();
        }
        // draw user desired position
        glColor3f(0,0,1); // Set current color
        glPushMatrix();
        glTranslatef(0,x_d1,0);
        gluDisk(q, 0, 0.07, 30, 1);
        glPopMatrix();
      }

      // if visual feedback is active, display force vector
      if (showforce) {
        double sign = (Fenv > 0) - (Fenv < 0); // sign of force vector
        if (sign != 0) {
          glLineWidth(2);
          glColor3f(1,1,1);
          glBegin(GL_LINES);
          glVertex2f(0,x_a);
          glVertex2f(0,x_a + Fenv*0.01);
          glEnd();
          glLineWidth(1);
          glPushMatrix();
          glTranslatef(0,x_a + Fenv*0.01,0);
          gluDisk(q, 0, 0.015, 30, 1);
          glPopMatrix();
        }
      }
      break;

    case waitingForUser:
      ROS_DEBUG("display waitingForUser messsage");
      // display number of trials completed
      x = -0.6;
      y = 0.3; 
      if (set_type == practice) {
        sprintf(mystring,"Practice trial %i of %i complete.",trial_number-1,nTrials);
      } else if (set_type == experiment) {
        sprintf(mystring,"Experiment trial %i of %i complete.",trial_number-1,nTrials);
      } else if (set_type == training) {
        sprintf(mystring,"Training trial %i of %i complete.",trial_number-1,nTrials);
      } else {
        sprintf(mystring,"ERROR: Unknown set type.");
      }
      glRasterPos2f(x,y);
      print_bitmap_string(GLUT_BITMAP_HELVETICA_18,mystring);
      y -= 0.1;
      glRasterPos2f(x,y);
      sprintf(mystring,"Press enter to begin next trial.");
      print_bitmap_string(GLUT_BITMAP_HELVETICA_18,mystring);
      y -= 0.1;
      break;

    case betweenSets:
      ROS_DEBUG("display betweenSets message");
      // display time remaining on break
      x = -1.0;
      y = 0.3; 
      sprintf(mystring,"Set complete. Please take a break at this time,");
      glRasterPos2f(x,y);
      print_bitmap_string(GLUT_BITMAP_HELVETICA_18,mystring);
      y -= 0.1;
      sprintf(mystring,"and notify the experimenter.");
      glRasterPos2f(x,y);
      print_bitmap_string(GLUT_BITMAP_HELVETICA_18,mystring);
      y -= 0.1;
      break;

    case betweenTraining:
      ROS_DEBUG("display betweenTraining message");
      // display time remaining on break
      x = -0.7;
      y = 0.3; 
      sprintf(mystring,"Set complete. Please take a break at this time,");
      glRasterPos2f(x,y);
      print_bitmap_string(GLUT_BITMAP_HELVETICA_18,mystring);
      y -= 0.1;
      sprintf(mystring,"and notify the experimenter.");
      glRasterPos2f(x,y);
      print_bitmap_string(GLUT_BITMAP_HELVETICA_18,mystring);
      y -= 0.1;
      break;

    case instructions:
      ROS_DEBUG("display task instructions");
      // give user instructions about the task
      // not needed for this experiment, since there is only one task
      /*
      if (task == forcemin) {
        sprintf(mystring,"In this experiment, the goal is to control your ball to");
        glRasterPos2f(x,y);
        print_bitmap_string(GLUT_BITMAP_HELVETICA_18,mystring);
        y -= 0.1;
        sprintf(mystring,"minimize the length of the attached arrow by moving in");
        glRasterPos2f(x,y);
        print_bitmap_string(GLUT_BITMAP_HELVETICA_18,mystring);
        y -= 0.1;
        sprintf(mystring,"the direction of the arrow.");
        glRasterPos2f(x,y);
        print_bitmap_string(GLUT_BITMAP_HELVETICA_18,mystring);
        y -= 0.1;
      } else if (task == tracking) {
        sprintf(mystring,"In this experiment, the goal is to control your ball to");
        glRasterPos2f(x,y);
        print_bitmap_string(GLUT_BITMAP_HELVETICA_18,mystring);
        y -= 0.1;
        sprintf(mystring,"follow the curve on the screen as closely as possible.");
        glRasterPos2f(x,y);
        print_bitmap_string(GLUT_BITMAP_HELVETICA_18,mystring);
        y -= 0.1;
      } else {
        sprintf(mystring,"ERROR: Unknown task type.");
        glRasterPos2f(x,y);
        print_bitmap_string(GLUT_BITMAP_HELVETICA_18,mystring);
        y -= 0.1;
      }
      */
      // TODO: support for multiple concurrent feedback types
      y = 0.3; 
      if (set_type == training) {
        x = -0.9;
        sprintf(mystring,"This set will be a training set, consisting of %i trials.", nTrials);
        glRasterPos2f(x,y);
        print_bitmap_string(GLUT_BITMAP_HELVETICA_18,mystring);
        y -= 0.1;
        if (visual) {
          sprintf(mystring,"In this set, you will receive visual feedback. Press");
          glRasterPos2f(x,y);
          print_bitmap_string(GLUT_BITMAP_HELVETICA_18,mystring);
          y -= 0.1;
          sprintf(mystring,"enter to begin. When you are finished, press enter");
          glRasterPos2f(x,y);
          print_bitmap_string(GLUT_BITMAP_HELVETICA_18,mystring);
          y -= 0.1;
          sprintf(mystring,"again to end.");
          glRasterPos2f(x,y);
          print_bitmap_string(GLUT_BITMAP_HELVETICA_18,mystring);
          y -= 0.1;
        } else if (vibrotactile) {
          sprintf(mystring,"In this set, you will receive vibration feedback.");
          glRasterPos2f(x,y);
          print_bitmap_string(GLUT_BITMAP_HELVETICA_18,mystring);
          y -= 0.1;
          sprintf(mystring,"Press enter to begin. When you are finished,");
          glRasterPos2f(x,y);
          print_bitmap_string(GLUT_BITMAP_HELVETICA_18,mystring);
          y -= 0.1;
          sprintf(mystring,"press enter again to end.");
          glRasterPos2f(x,y);
          print_bitmap_string(GLUT_BITMAP_HELVETICA_18,mystring);
          y -= 0.1;
        } else if (skin_stretch) {
          sprintf(mystring,"In this set, you will receive skin stretch feedback.");
          glRasterPos2f(x,y);
          print_bitmap_string(GLUT_BITMAP_HELVETICA_18,mystring);
          y -= 0.1;
          sprintf(mystring,"Press enter to begin. When you are finished, press");
          glRasterPos2f(x,y);
          print_bitmap_string(GLUT_BITMAP_HELVETICA_18,mystring);
          y -= 0.1;
          sprintf(mystring,"enter again to end.");
          glRasterPos2f(x,y);
          print_bitmap_string(GLUT_BITMAP_HELVETICA_18,mystring);
          y -= 0.1;
        } else {
          sprintf(mystring,"ERROR: No feedback type specified.");
          glRasterPos2f(x,y);
          print_bitmap_string(GLUT_BITMAP_HELVETICA_18,mystring);
          y -= 0.1;
          sprintf(mystring,"Press enter to begin. When you are finished,");
          glRasterPos2f(x,y);
          print_bitmap_string(GLUT_BITMAP_HELVETICA_18,mystring);
          y -= 0.1;
          sprintf(mystring,"press enter again to end.");
          glRasterPos2f(x,y);
          print_bitmap_string(GLUT_BITMAP_HELVETICA_18,mystring);
          y -= 0.1;
        }
      } else if ((set_type == practice) || (set_type == experiment)) {
        x = -1.0;
        if (set_type == practice) {
          sprintf(mystring,"This set will be a practice set, consisting of %i trials of %d", nTrials, trial_length);
        } else if (set_type == experiment) {
          sprintf(mystring,"This set will be an experiment set, consisting of %i trials of %d", nTrials, trial_length);
        }
        glRasterPos2f(x,y);
        print_bitmap_string(GLUT_BITMAP_HELVETICA_18,mystring);
        y -= 0.1;
        if (visual) {
          sprintf(mystring,"seconds each. In this set, you will receive visual feedback.");
        } else if (vibrotactile) {
          sprintf(mystring,"seconds each. In this set, you will receive vibration feedback.");
        } else if (skin_stretch) {
          sprintf(mystring,"seconds each. In this set, you will receive skin stretch");
        } else {
          sprintf(mystring,"ERROR: No feedback type specified.");
        }
        glRasterPos2f(x,y);
        print_bitmap_string(GLUT_BITMAP_HELVETICA_18,mystring);
        y -= 0.1;
        if (skin_stretch) {
          sprintf(mystring,"feedback. Press enter to begin.");
          glRasterPos2f(x,y);
          print_bitmap_string(GLUT_BITMAP_HELVETICA_18,mystring);
          y -= 0.1;
        } else {
          sprintf(mystring,"Press enter to begin.");
          glRasterPos2f(x,y);
          print_bitmap_string(GLUT_BITMAP_HELVETICA_18,mystring);
          y -= 0.1;
        }
      } else {
        x = -0.9;
        sprintf(mystring,"ERROR: Unknown set type.");
        glRasterPos2f(x,y);
        print_bitmap_string(GLUT_BITMAP_HELVETICA_18,mystring);
        y -= 0.1;
        if (visual) {
          sprintf(mystring,"In this set, you will receive visual feedback.");
        } else if (vibrotactile) {
          sprintf(mystring,"In this set, you will receive vibration feedback.");
        } else if (skin_stretch) {
          sprintf(mystring,"In this set, you will receive skin stretch");
        } else {
          sprintf(mystring,"ERROR: No feedback type specified.");
        }
        glRasterPos2f(x,y);
        print_bitmap_string(GLUT_BITMAP_HELVETICA_18,mystring);
        y -= 0.1;
        if (skin_stretch) {
          sprintf(mystring,"feedback. Press enter to begin.");
          glRasterPos2f(x,y);
          print_bitmap_string(GLUT_BITMAP_HELVETICA_18,mystring);
          y -= 0.1;
        } else {
          sprintf(mystring,"Press enter to begin.");
          glRasterPos2f(x,y);
          print_bitmap_string(GLUT_BITMAP_HELVETICA_18,mystring);
          y -= 0.1;
        }
      }
      break;

    case rating:
      x = -0.9;
      y = 0.3; 
      ROS_DEBUG("display rating instructions");
      // give user instructions on selecting a rating (1-10)
      sprintf(mystring,"Please rate the difficulty of this set from 1 (Easiest) to 5");
      glRasterPos2f(x,y);
      print_bitmap_string(GLUT_BITMAP_HELVETICA_18,mystring);
      y -= 0.1;
      sprintf(mystring,"(Most Difficult) by pressing the number on the keyboard.");
      glRasterPos2f(x,y);
      print_bitmap_string(GLUT_BITMAP_HELVETICA_18,mystring);
      y -= 0.1;
      break;

    case experimentDone:
      x = -0.6;
      y = 0.3; 
      ROS_DEBUG("display done instructions");
      // display done message
      sprintf(mystring,"Done. Please notify experimenter.");
      glRasterPos2f(x,y);
      print_bitmap_string(GLUT_BITMAP_HELVETICA_18,mystring);
      y -= 0.1;
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
  exit(1);
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
    // publish quit message for all nodes
    ROS_DEBUG("Experiment quit key received.");
    std_msgs::Bool msg;
    msg.data = true;
    quit_pub->publish(msg);
    quit();
  } else if (key == KEY_Q) {
    // just quit graphics node
    ROS_DEBUG("Graphics quit key received.");
    quit();
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
	glutTimerFunc(30,update,0);
  ros::spinOnce();
}


void experimentStateCallback(const simulation_impedance_experiment::ExperimentState::ConstPtr& msg) {
  task = (taskType)(msg->task);
  x_a = msg->x_a;
  x_d1 = msg->x_d1;
  x_d2 = msg->x_d2;
  Fenv = msg->Fenv;
  direction = msg->direction;
  nProjectiles = msg->nProjectiles;
  x_p = msg->x_p;
}

void graphicsInfoCallback(const simulation_impedance_experiment::GraphicsInfo::ConstPtr& msg) {
  limitL = msg->limitL;
  limitR = msg->limitR;
  phase = msg->phase;
}

bool setMode(simulation_impedance_experiment::SetGraphicsMode::Request  &req,
             simulation_impedance_experiment::SetGraphicsMode::Response &res )
{
  showforce = req.showforce;
  debug_mode = req.debug_mode;
  return true;
}


bool changeState(simulation_impedance_experiment::ChangeState::Request  &req,
                 simulation_impedance_experiment::ChangeState::Response &res )
{
  state = (stateT)(req.state);
  trial_number = req.trial_number;
  nTrials = req.nTrials;
  trial_length = req.trial_length;
  set_type = (setType)(req.set_type);
  visual = req.visual;
  vibrotactile = req.vibrotactile;
  skin_stretch = req.skin_stretch;
  return true;
}


int main(int argc, char **argv) {

  // set up ROS stuff
  ros::init(argc, argv, "graphics");
  //ros::NodeHandle n;
  n = new ros::NodeHandle;
  ros::Time t;
  ros::Time told;
  ros::Duration deltat;

  // subscribe to experiment state to get info for graphics
  ros::Subscriber sub_experiment_state = n->subscribe<simulation_impedance_experiment::ExperimentState>("experiment_state", 1000,experimentStateCallback);
  ros::Subscriber sub_graphics_info = n->subscribe<simulation_impedance_experiment::GraphicsInfo>("graphics_info", 1000,graphicsInfoCallback);
  quit_pub = new ros::Publisher;
  *quit_pub = n->advertise<std_msgs::Bool>("quit", 1000);
  key_press_pub = new ros::Publisher;
  *key_press_pub = n->advertise<std_msgs::Int32>("key_press",1000);

  // advertise services for things that don't happen often
  ros::ServiceServer server_set_graphics_mode = n->advertiseService("set_graphics_mode", setMode);
  ros::ServiceServer server_change_state = n->advertiseService("change_state", changeState);

  if (ros::ok()) {

    //Set up glut
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE); // Color display mode, RGB, and A = alpha, which defines opacity; also activate double buffering
    glutInitWindowSize(WIN_WIDTH, WIN_HEIGHT); // Set window size
    glutInitWindowPosition(0, 0); // Window placement. (0,0) corresponds to top left of screen.
    glutCreateWindow("Experiment!"); // Creates the window and gives it a name
    glutFullScreen();
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
  delete n;

  return 0;
}
