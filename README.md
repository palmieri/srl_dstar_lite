# srl_dstar_lite

### Description of the Package
srl_dstar_lite is a ROS move_base package that implements D* Lite. The latter is a common motion planning algorithm that deals with unexpected obstacles and quickly react to dynamic world's changes.  

The package is a global_planner plug-in for move_base. It adhers to the specifics of nav_core::BaseGlobalPlanner, please check for further details on move_base refer to http://wiki.ros.org/move_base.

### Requirements
* ROS (including visualization rools -> rviz), tested on Indigo, Hydro and Kinetic
* ros-hydro-navigation or ros-indigo-navigation or ros-kinetic-navigation
* Eigen3
* Boost >= 1.46
* C++11 compiler

### Installation

Clone the package into you catkin workspace
- `cd [workspace]/src`
- `git clone https://github.com/laujinseoi/srl_dstar_lite.git`
- `cd ../`
- `catkin_make` or `catkin build`

### Usage
- Change your ros param name "base_global_planner" to "srl_dstar_lite/SrlDstarLiteROS", then launch your navigation stack, and it will work!

### Developers
Any contribution to the software is welcome. Contact the current developers for any info: 
* Luigi Palmieri (https://github.com/palmieri, palmieri(at)informatik.uni-freiburg.de)

### TODOs:
* Rewrite the code to be conform to the ROS Cpp style guide, see http://wiki.ros.org/CppStyleGuide
* Use dynamic reconfiguration of the parameters, see http://wiki.ros.org/dynamic_reconfigure
* Improve documentation

