_Under construction!_

# Introduction #

This package contains the code needed to run the virtual prosthesis experiment associated with the following paper submitted to BioRob 2012:

A. Blank, A. M. Okamura, and L. L. Whitcomb. User comprehension of task performance with varying impedance in a virtual prosthetic arm. IEEE International Conference on Biomedical Robotics and Biomechatronics, 2012. Accepted.

## Table of Contents ##


# Requirements #

Hardware:
  * Two data acquisition cards with analog input and output, for example, [here](http://www.mccdaq.com/pci-data-acquisition/PCI-DAS6014.aspx). (Code can be easily modified to use one card, if it has enough output pins.)

Software:
  * [Comedi](http://www.comedi.org)
  * [OpenGL](http://www.opengl.org) with [GLUT](http://www.opengl.org/resources/libraries/glut/)
  * [GSL](http://www.gnu.org/s/gsl/)

# Download #

Export the code from svn:
```
svn export http://jhu-lcsr-ros-pkg.googlecode.com/svn/pkgs/trunk/simulation_impedance_experiment simulation_impedance_experiment
```

Or clone the entire repo through mercurial:
```
hg clone https://jhu-lcsr-ros-pkg.googlecode.com/hg/packages/ jhu_lcsr_ros_pkg
```

# Build #

Make sure the necessary dependencies are installed:
```
rosdep install simulation_impedance_experiment
```

Then build the package:
```
rosmake simulation_impedance_experiment
```

# Run #

The experiment uses four nodes, which should be started in this order:

  * daq - interfaces with data acquisition card for reading force sensor input and providing tactor output
  * simulation - simulates 1-dof virtual prosthesis and handles data logging
  * graphics - provides visual feedback and instructions to the experiment subject, and handles keyboard input
  * experiment\_manager - handles experiment trials, selects feedback methods, etc.

Experiment data is logged by the simulation node, and written into m-files at the end of each trial. Data files are located in data/Subjectxx/ and are named with the set and trial number as Setxx\_Trialxx.m. Rating data written at the end of the experiment into a single m-file, data/Subjectxx/all\_ratings.m.

# Support #

For more information, email ablanket@gmail.com.