/*
    SRL D* Lite ROS Package
    Copyright (C) 2015, Palmieri Luigi, palmieri@informatik.uni-freiburg.de

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>

 */

#ifndef DSTAR_PLANNER_CPP
#define DSTAR_PLANNER_CPP

#include <cstdlib>
#include <fstream>
#include <iostream>

#include <ros/console.h>
#include <ros/ros.h>

#include <tf/transform_listener.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <std_msgs/Bool.h>
#include <std_msgs/Header.h>

#include <nav_msgs/GridCells.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <geometry_msgs/Quaternion.h>

#include <srl_dstar_lite/costmap_model.h>
#include <srl_dstar_lite/data_structures.h>
#include <srl_dstar_lite/dstar.h>
#include <srl_dstar_lite/pathSplineSmoother/pathSplineSmoother.h>
#include <srl_dstar_lite/world_model.h>

#include <costmap_2d/cost_values.h>
#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/costmap_2d_ros.h>

#include <nav_core/base_global_planner.h>

#define COST_POSSIBLY_CIRCUMSCRIBED 128

namespace srl_dstar_lite
{

///<  @brief SrlDstarLite class
class SrlDstarLite : public nav_core::BaseGlobalPlanner
{

private:
  ros::NodeHandle nh_;
  // Publishers
  ros::Publisher pub_path_;
  ros::Publisher pub_goal_;
  ros::Publisher pub_tree_;
  ros::Publisher pub_tree_dedicated_;
  ros::Publisher pub_path_dedicated_;
  ros::Publisher pub_samples_;
  ros::Publisher pub_graph_;
  ros::Publisher pub_no_plan_;
  ros::Publisher pub_obstacle_markers_;

  ros::Subscriber sub_obstacles_;
  ros::Subscriber sub_all_agents_;

public:
  /**
  * @brief Constructor of the Srl_global_planner
  * @param node, Ros NodeHandle
  * @param world_model, Cost Map model to load informaiton of the Global Cost map
  * @param footprint_spec, footprint of the robot
  * @param costmap_ros, cost_map ros wrapper
  */
  SrlDstarLite() : costmap_ros_(NULL), initialized_(false), world_model_(NULL) {}

  /**
  * @brief Constructor of the Srl_global_planner
  * @param name, name to associate to the node
  * @param costmap_ros, costmap_ros
  */
  SrlDstarLite(std::string name, costmap_2d::Costmap2DROS* costmap_ros) : costmap_ros_(costmap_ros)
  {

    initialize(name, costmap_ros);
  }

  /**
  * @brief Initialize the ros handle
  * @param name, Ros NodeHandle name
  * @param costmap_ros, cost_map ros wrapper
  */
  void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

  /**
  * @brief makePlan, follows the virtual method of the base class
  * @param start, Start pose
  * @param goal, goal pose
  * @param plan, generated path
  * @return bool, true
  */
  bool makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,
                std::vector< geometry_msgs::PoseStamped >& plan);

  /**
  * @brief callbackObstacles, Read occupancy grid from global cost map
  * @return void
  */
  void callbackObstacles(const nav_msgs::OccupancyGrid::ConstPtr& msg);

  /**
  * @brief publishPath, Publish path
  * @return void
  */
  void publishPath(std::vector< geometry_msgs::PoseStamped > grid_plan);

  /**
  * @brief plan a kinodynamic path using RRT
  * @param, grid_plan Plan generated
  * @param, start pose
  * @return true, if the plan was found
  */
  int plan(std::vector< geometry_msgs::PoseStamped >& grid_plan, geometry_msgs::PoseStamped& start);

  /**
  * @brief set the Goal region
  * @param, x coordinate of the goal pose
  * @param, y coordinate of the goal pose
  * @param, theta yaw angle of the goal pose
  * @param, toll yaw angle of the goal pose
  * @param, goal_frame goal frame
  * @return void
  */
  void setGoal(double x, double y, double theta, double toll, std::string goal_frame);

  /**
  * @brief Transform pose in planner_frame
  * @param, init_pose initial pose to transform
  * @return Pose transformed
  */
  geometry_msgs::PoseStamped transformPose(geometry_msgs::PoseStamped init_pose);

  /**
  * @brief Wrap the angle
  * @param, alpha, angle to wrap in the range [min min+2*M_PI]
  * @param, min, beginning of the range
  * @return angle in correct range
  */
  double set_angle_to_range(double alpha, double min);

  /**
  * @brief Get from the cost map the footprint cost associated to the point x_bot, y_bot, orient_bot
  * @param,  x_bot, y_bot, orient_bot, robot pose
  * @return  footprint cost
  */

  double getFootPrintCost(double x_bot, double y_bot, double orient_bot);

  /**
  * @brief Smooth Plan, smooth the path using splines
  * @param,  list<state> path, path generated by the D* Lite algorithm
  * @return  the path shortcutted
  */
  vector< RealPoint > SmoothPlan(list< state > path);

  /**
  * @brief ShortcutPlan, shortcuts the path if no obstacles is between the points
  * @param,  list<state> path, path generated by the D* Lite algorithm
  * @return  the path shortcutted
  */
  list< state > ShortcutPlan(list< state > path);

  bool initialized_;

  int cnt_make_plan_;

  double cellwidth_; ///<  @brief Cell width

  double cellheight_; ///<  @brief Cell height

  base_local_planner::CostmapModel* world_model_; ///<  @brief World Model associated to the costmap

  std::vector< geometry_msgs::Point > footprint_spec_; ///< @brief FootPrint list of points of the robot

  costmap_2d::Costmap2DROS* costmap_ros_; ///< @brief The ROS wrapper for the costmap the controller will use

  costmap_2d::Costmap2D* costmap_; ///< @brief The ROS wrapper for the costmap the controller will use

  std::string node_name_; ///<  @brief Node name

  double goal_x_; ///<  @brief x coordinate of the goal pose

  double goal_y_; ///<  @brief y coordinate of the goal pose

  double goal_theta_; ///<  @brief yaw angle of the goal pose

  double toll_goal_; ///<  @brief toll the goal region

  double rx, ry, rz; ///<  @brief robot coordinates

  double xscene_; ///<  @brief Width of the scene in meters

  double yscene_; ///<  @brief Height of the scene in meters

  std::vector< Tobstacle > obstacle_positions; ///<  @brief Obstacles

  std::vector< Thuman > agents_position; ///<  @brief agents

  bool goal_init_; ///<  @brief Flag that indicates if the goal is initialized

  geometry_msgs::Pose start_pose_; ///<  @brief Start pose

  geometry_msgs::Pose goal_pose_; ///<  @brief Goal pose

  //
  ros::Time begin; ///<  @brief fMap loading only during the first Nsecs

  double initialization_time; ///<  @brief initialization time for map

  double map_loading_time; ///<  @brief map loading time

  double max_map_loading_time; ///<  @brief max map loading time

  int cnt_map_readings; ///<  @brief counter of map readings

  std::string costmap_frame_; ///<  @brief costmap frame

  std::string planner_frame_; ///<  @brief planner frame

  double width_map_; ///<  @brief Width of the 2D space where to sample

  double height_map_; ///<  @brief Height of the 2D space where to sample

  double center_map_x_; ///<  @brief x coordinate of the center of 2D space where to sample

  double center_map_y_; ///<  @brief y coordinate of the center of 2D space where to sample

  int cnt_no_plan_; ///<  @brief counter of no planning sol

  Dstar* dstar_planner_; ///<  Dstar planner

  PathSplineSmoother* spline_smoother_;

  bool SMOOTHING_ON_;

  bool SHORTCUTTING_ON_;
};
}

#endif // DSTAR_planner_H
