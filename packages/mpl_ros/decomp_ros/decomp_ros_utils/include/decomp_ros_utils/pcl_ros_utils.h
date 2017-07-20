#ifndef PCL_ROS_UTILS_H
#define PCL_ROS_UTILS_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <sensor_msgs/PointCloud.h>

typedef pcl::PointXYZ PCLPoint;
typedef pcl::PointCloud<PCLPoint> PCLPointCloud;

namespace PCLROSUtils
{
  PCLPointCloud toPCL(const sensor_msgs::PointCloud &cloud_ros);
  sensor_msgs::PointCloud toROS(const PCLPointCloud& cloud_pcl);
}
#endif

