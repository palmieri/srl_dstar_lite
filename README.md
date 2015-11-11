# srl_dstar_lite

### Description of the Package
srl_dstar_lite is a ROS move_base package that implements D* Lite. The latter is a common motion planning algorithm that deals with unexpected obstacles and quickly react to dynamic world's changes.  

The package is a global_planner plug-in for move_base. It adhers to the specifics of nav_core::BaseGlobalPlanner, please check for further details on move_base refer to http://wiki.ros.org/move_base.

### Requirements
* ROS (including visualization rools -> rviz), tested on Indigo and Hydro
* ros-hydro-navigation or ros-indigo-navigation
* Eigen3
* Boost >= 1.46
* C++11 compiler

### Installation

Clone the package into you catkin workspace
- `cd [workspace]/src`
- `git clone https://github.com/palmieri/srl_dstar_lite.git`
- `cd ../`
- `catkin_make` or `catkin build`

### Usage
- 

### Developers
Any contribution to the software is welcome. Contact the current developers for any info: 
* Luigi Palmieri (https://github.com/palmieri, palmieri(at)informatik.uni-freiburg.de)

### TODOs:
* Rewrite the code to be conform to the ROS Cpp style guide, see http://wiki.ros.org/CppStyleGuide
* Use dynamic reconfiguration of the parameters, see http://wiki.ros.org/dynamic_reconfigure
* Improve documentation

