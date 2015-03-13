_Under construction!_

# Introduction #

This package contains the code needed to run the WAM experiment associated with the following doctoral dissertation:

A. Blank. Proprioceptive Motion Feedback and User-Selectable Impedance for Improved Upper-Limb Prosthesis Control. Johns Hopkins University, 2012.

## Table of Contents ##


# Requirements #

Hardware:
  * One data acquisition cards with analog input, for example, [here](http://www.mccdaq.com/pci-data-acquisition/PCI-DAS6014.aspx).
  * Ubuntu with Xenomai (see installation instructions [here](UbuntuXenomai.md)).


Software:
  * [Comedi](http://www.comedi.org)
  * [OpenGL](http://www.opengl.org) with [GLUT](http://www.opengl.org/resources/libraries/glut/)
  * [GSL](http://www.gnu.org/s/gsl/)

# Download #

Clone the entire repo through mercurial:
```
hg clone https://jhu-lcsr-ros-pkg.googlecode.com/hg/packages/ jhu_lcsr_ros_pkg
```

# Build #

Make sure the necessary dependencies are installed:
```
rosdep install wam_impedance_experiment
```

Then build the package:
```
rosmake wam_impedance_experiment
```

# Run #

To start the main experiment nodes, use the launch file experiment.launch, as described in the README file. Start the graphics node separately with the rosrun command. This allows the graphics to be run on a different machine (provided the communication is set up correctly).

Experiment data is logged by the logger node and written into m-files at the end of each trial. Data files are located in data/Subjectxx/ and are named with the set and trial number as Setxx\_Trialxx.m.

# Support #

For more information, email ablanket@gmail.com.