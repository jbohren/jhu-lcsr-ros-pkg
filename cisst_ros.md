## Introduction ##
The cisst\_ros stack currently provides two functionalities:
  * A set of ROS packages wrapping the [CISST libraries](https://trac.lcsr.jhu.edu/cisst)
  * ROS packages that aide in the integration of CISST components with ROS

<span><i><b>NOTE:</b> cisst_ros is still "experimental" and under volatile development</i></span>

## Table of Contents ##


## Download & Install ##
The cisst\_ros stack can be installed locally by checking out the stack directly from svn.

### Releases ###

| **Version** | **Checkout Command** |
|:------------|:---------------------|
| tip | `hg clone https://code.google.com/p/jhu-lcsr-ros-pkg` |

### For Developers ###
See [Checking out for Development](http://code.google.com/p/jhu-lcsr-ros-pkg/wiki/Development?ts=1309236926&updated=Development#Checking_Out_for_Development). To browse this stack in the svn repository, click [here](http://code.google.com/p/jhu-lcsr-ros-pkg/source/browse/#svn%2Fstacks%2Fcisst_ros%2Ftrunk).

## Building ##
After checking out a version of the cisst\_ros package, make sure all the necessary system dependencies are installed by executing:
```
rosdep install cisst
```
Once the system dependencies are installed, you can build the cisst and cisst\_ros\_integration packages by executing:
```
rosmake cisst_ros
```

## Useful CISST-ROS Interfaces (Current and Planned) ##
![http://jhu-lcsr-ros-pkg.googlecode.com/svn/stacks/cisst_ros/trunk/cisst_ros_integration/doc/cisst_ros_translation.png](http://jhu-lcsr-ros-pkg.googlecode.com/svn/stacks/cisst_ros/trunk/cisst_ros_integration/doc/cisst_ros_translation.png)