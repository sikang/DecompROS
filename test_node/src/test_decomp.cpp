#include "bag_reader.hpp"
#include "txt_reader.hpp"
#include <decomp_ros_utils/data_ros_utils.h>
#include <ros/ros.h>
#include <decomp_util/ellipse_decomp.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <nav_msgs/Path.h>

std_msgs::Header header_;

int main(int argc, char ** argv){
  ros::init(argc, argv, "test");
  ros::NodeHandle nh("~");

  ros::Publisher map_pub = nh.advertise<sensor_msgs::PointCloud2>("cloud", 1, true);
  ros::Publisher marker_pub = nh.advertise<visualization_msgs::MarkerArray>("markers", 1, true);
  ros::Publisher path_pub = nh.advertise<nav_msgs::Path>("path", 1, true);
  ros::Publisher es_pub = nh.advertise<decomp_ros_msgs::Ellipsoids>("ellipsoids", 1, true);
  ros::Publisher poly_pub = nh.advertise<decomp_ros_msgs::Polyhedra>("polyhedra", 1, true);

  header_.frame_id = std::string("map");
  std::string file_name, topic_name, path_file;

  nh.param("path_file", path_file, std::string("path.txt"));
  nh.param("bag_file", file_name, std::string("voxel_map"));
  nh.param("bag_topic", topic_name, std::string("voxel_map"));
  sensor_msgs::PointCloud2 map = read_point_cloud2(file_name, topic_name);
  map.header = header_;
  map_pub.publish(map);

  sensor_msgs::PointCloud cloud;
  sensor_msgs::convertPointCloud2ToPointCloud(map, cloud);
  vec_Vec3f obs = cloud_to_vec(cloud);

  /*
  visualization_msgs::MarkerArray markers = read_marker_array(file_name, topic_name);
  for(auto & it: markers.markers)
    it.header = header_;
  marker_pub.publish(markers);
  */
  vec_Vec3f path;
  if(!read_path(path_file, path))
    ROS_ERROR("Fail to read a path!");

  nav_msgs::Path path_msg = eigen_to_path(path);
  path_msg.header = header_;
  path_pub.publish(path_msg);

  EllipseDecomp decomp_util(Vec3f(-100, -100, -5), Vec3f(200, 200, 10), true);
  decomp_util.set_obstacles(obs);
  decomp_util.decomp(path);

  decomp_ros_msgs::Ellipsoids es_msg = ellipsoids_to_ros(decomp_util.get_ellipsoids());
  es_msg.header = header_;
  es_pub.publish(es_msg);

  decomp_ros_msgs::Polyhedra poly_msg = polyhedra_to_ros(decomp_util.get_polyhedra());
  poly_msg.header = header_;
  poly_pub.publish(poly_msg);


  vec_LinearConstraint3f cs = decomp_util.get_constraints();
  for(int i = 0; i < cs.size(); i++) {
    MatD3f A = cs[i].first;
    VecDf b = cs[i].second;

    printf("i: %d\n", i);
    std::cout << "start: " << (A*path[i]-b).transpose() << std::endl;
    std::cout << "end: " << (A*path[i+1]-b).transpose() << std::endl;

    /*
    std::cout << A.row(0) << std::endl;
    std::cout << b.row(0) << std::endl;
    std::cout << path[0].transpose() << std::endl;
    */
  }


  ros::spin();

  return 0;
}
