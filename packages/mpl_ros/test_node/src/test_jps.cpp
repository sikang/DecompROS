#include "bag_reader.hpp"
#include <ros_utils/data_ros_utils.h>
#include <jps3d/yag_planner/a_star_util.h>
#include <jps3d/yag_planner/jps_2d_util.h>
#include <jps3d/yag_planner/jps_3d_util.h>
#include <jps3d/nx_planner/nx_jps_3d.h>

using namespace JPS;

void setMap(JPS::VoxelMapUtil* map_util, const planning_ros_msgs::VoxelMap& msg) {
  Vec3f ori(msg.origin.x, msg.origin.y, msg.origin.z);
  Vec3i dim(msg.dim.x, msg.dim.y, msg.dim.z);
  decimal_t res = msg.info.resolution;
  std::vector<signed char> map = msg.data;

  map_util->setMap(ori, dim, map, res);
}

void getMap(JPS::VoxelMapUtil* map_util, planning_ros_msgs::VoxelMap& map) {
  Vec3f ori = map_util->getOrigin();
  Vec3i dim = map_util->getDim();
  decimal_t res = map_util->getRes();

  map.origin.x = ori(0);
  map.origin.y = ori(1);
  map.origin.z = ori(2);

  map.dim.x = dim(0);
  map.dim.y = dim(1);
  map.dim.z = dim(2);
  map.info.resolution = res;

  map.data = map_util->getMap();
}

int main(int argc, char ** argv){
  ros::init(argc, argv, "test");
  ros::NodeHandle nh("~");

  ros::Publisher map_pub = nh.advertise<planning_ros_msgs::VoxelMap>("voxel_map", 1, true);
  ros::Publisher path_pub1 = nh.advertise<planning_ros_msgs::Path>("path1", 1, true);
  ros::Publisher path_pub2 = nh.advertise<planning_ros_msgs::Path>("path2", 1, true);
  ros::Publisher path_pub3 = nh.advertise<planning_ros_msgs::Path>("path3", 1, true);

  ros::Time t0 = ros::Time::now();
  //Read map from bag file
  std::string file_name, topic_name;
  nh.param("file", file_name, std::string("voxel_map"));
  nh.param("topic", topic_name, std::string("voxel_map"));
  planning_ros_msgs::VoxelMap map = read_bag(file_name, topic_name);

  //Initialize map util 
  std::shared_ptr<VoxelMapUtil> map_util(new VoxelMapUtil);
  setMap(map_util.get(), map);

  double robot_radius;
  nh.param("robot_r", robot_radius, 0.5);
  //Free unknown space and dilate obstacles
  map_util->freeUnKnown();
  map_util->dilate(robot_radius, 0.0);
  map_util->dilating();


  //Publish the dilated map for visualization
  getMap(map_util.get(), map);
  map_pub.publish(map);

  ROS_INFO("Takse %f sec to set up map!", (ros::Time::now() - t0).toSec());

  Vec3f start, goal;
  nh.param("start_x", start(0), 0.0);
  nh.param("start_y", start(1), 0.0);
  nh.param("start_z", start(2), 0.0);

  nh.param("goal_x", goal(0), 0.0);
  nh.param("goal_y", goal(1), 0.0);
  nh.param("goal_z", goal(2), 0.0);

  std::unique_ptr<PlannerBase> planner_jps(new JPS3DUtil(false)); // Declare a jps planner
  planner_jps->setMapUtil(map_util.get()); // Set collision checking function
  t0 = ros::Time::now();
  bool valid_jps = planner_jps->plan(start, goal);
  ROS_INFO("Takse %f sec to plan using jps!", (ros::Time::now() - t0).toSec());

  std::unique_ptr<PlannerBase> planner_astar(new AStarUtil(false)); // Declare a A* planner
  planner_astar->setMapUtil(map_util.get()); // Set collision checking function

  t0 = ros::Time::now();
  bool valid_astar = planner_astar->plan(start, goal); // Plan from start to goal
  ROS_INFO("Takse %f sec to plan using a star!", (ros::Time::now() - t0).toSec());

  std::unique_ptr<PlannerBase> planner_nx(new NXJPS3DUtil(false)); // Declare a A* planner
  planner_nx->setMapUtil(map_util.get()); // Set collision checking function

  t0 = ros::Time::now();
  bool valid_nx = planner_nx->plan(start, goal); // Plan from start to goal
  ROS_INFO("Takse %f sec to plan using nx jps!", (ros::Time::now() - t0).toSec());


  if(valid_jps) {
    planning_ros_msgs::Path path_msg = path_to_mav(planner_jps->getRawPath());
    path_msg.header.frame_id = "map";
    path_pub1.publish(path_msg);
  }

  if(valid_astar) {
    planning_ros_msgs::Path path_msg = path_to_mav(planner_astar->getRawPath());
    path_msg.header.frame_id = "map";
    path_pub2.publish(path_msg);
  }
 
  if(valid_nx) {
    planning_ros_msgs::Path path_msg = path_to_mav(planner_nx->getRawPath());
    path_msg.header.frame_id = "map";
    path_pub3.publish(path_msg);
  }
 
  //Initialize planner
  ros::spin();

  return 0;
}
