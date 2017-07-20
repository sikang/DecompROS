#include "bag_reader.hpp"
#include "txt_reader.hpp"
#include <decomp_ros_utils/data_ros_utils.h>
#include <ros/ros.h>
#include <decomp_util/seed_decomp.h>
#include <sensor_msgs/point_cloud_conversion.h>

#include <decomp_ros_utils/pcl_ros_utils.h>
#include <pcl/kdtree/kdtree_flann.h>

std_msgs::Header header_;

void near(const Vec3f& pt, double r, const vec_Vec3f& raw_obs, vec_Vec3f& obs) {
  for(const auto& it: raw_obs) {
    if((it - pt).norm() < r)
      obs.push_back(it);
  }
}

int main(int argc, char ** argv){
  ros::init(argc, argv, "test");
  ros::NodeHandle nh("~");

  ros::Publisher map_pub = nh.advertise<sensor_msgs::PointCloud2>("cloud", 1, true);
  ros::Publisher marker_pub = nh.advertise<visualization_msgs::MarkerArray>("markers", 1, true);
  ros::Publisher seed_pub = nh.advertise<sensor_msgs::PointCloud>("seed", 1, true);
  ros::Publisher es_pub = nh.advertise<decomp_ros_msgs::Ellipsoids>("ellipsoids", 1, true);
  ros::Publisher poly_pub = nh.advertise<decomp_ros_msgs::Polyhedra>("polyhedra", 1, true);

  header_.frame_id = std::string("map");
  std::string file_name, topic_name, marker_name, seed_file;

  nh.param("seed_file", seed_file, std::string("seed.txt"));
  nh.param("bag_file", file_name, std::string("voxel_map"));
  nh.param("bag_topic", topic_name, std::string("voxel_map"));
  nh.param("bag_marker", marker_name, std::string("voxel_map"));
  //Read the point cloud from bag
  sensor_msgs::PointCloud2 map = read_point_cloud2(file_name, topic_name);
  map.header = header_;
  map_pub.publish(map);

  //Convert into vector of Eigen
  sensor_msgs::PointCloud cloud;
  sensor_msgs::convertPointCloud2ToPointCloud(map, cloud);
  vec_Vec3f obs = DecompROS::cloud_to_vec(cloud);

  visualization_msgs::MarkerArray markers = read_marker_array(file_name, marker_name);
  for(auto & it: markers.markers)
    it.header = header_;
  marker_pub.publish(markers);

  PCLPointCloud::Ptr cloud_ptr = boost::make_shared<PCLPointCloud>(PCLROSUtils::toPCL(cloud));
  ros::Time t0 = ros::Time::now();
  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
  kdtree.setInputCloud (cloud_ptr);
  ROS_INFO("Takes %f sec to convert [%zu] points into kd tree", (ros::Time::now() - t0).toSec(), cloud.points.size());


  //Read path from txt
  vec_Vec3f seeds;
  if(!read_path(seed_file, seeds))
    ROS_ERROR("Fail to read seeds!");

  sensor_msgs::PointCloud seed_msg = DecompROS::vec_to_cloud(seeds);
  seed_msg.header = header_;
  seed_pub.publish(seed_msg);

  for(const auto & it: seeds) {


    pcl::PointXYZ searchPoint;
    searchPoint.x = it(0);
    searchPoint.y = it(1);
    searchPoint.z = it(2);

    float radius = 10.0;

    std::vector<int> pointIdxRadiusSearch;
    std::vector<float> pointRadiusSquaredDistance;

    ros::Time t1 = ros::Time::now();
    kdtree.radiusSearch (searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance);
    double dt_kd = (ros::Time::now() - t1).toSec();

    t1 = ros::Time::now();
    vec_Vec3f new_obs;
    near(it, radius, obs, new_obs);
    double dt_near = (ros::Time::now() - t1).toSec();
    printf("numbers: kdtree [%zu], normal: [%zu]\n", pointIdxRadiusSearch.size(), new_obs.size() );
    printf("times: kdtree [%f], normal: [%f]\n", dt_kd, dt_near );
  }
  /*
  vec_Ellipsoid es;
  Polyhedra polys;

  for(const auto& it: seeds) {
    //Using seed decomposition
    SeedDecomp decomp_util(it);
    decomp_util.set_obstacles(obs);
    decomp_util.dilate(5.0);
    es.push_back(decomp_util.ellipsoid());
    polys.push_back(decomp_util.polyhedron());
  }

  //Publish visualization msgs
  decomp_ros_msgs::Ellipsoids es_msg = DecompROS::ellipsoids_to_ros(es);
  es_msg.header = header_;
  es_pub.publish(es_msg);

  decomp_ros_msgs::Polyhedra poly_msg = DecompROS::polyhedra_to_ros(polys);
  poly_msg.header = header_;
  poly_pub.publish(poly_msg);
  */

  ros::spin();

  return 0;
}
