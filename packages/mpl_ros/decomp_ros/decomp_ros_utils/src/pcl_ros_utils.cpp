#include <decomp_ros_utils/pcl_ros_utils.h>

namespace PCLROSUtils
{

  PCLPointCloud toPCL(const sensor_msgs::PointCloud &cloud_ros)
  {
    PCLPointCloud cloud;
    cloud.width = cloud_ros.points.size();
    cloud.height = 1;
    cloud.points.resize(cloud.width * cloud.height);
    for(unsigned int i = 0; i < cloud_ros.points.size(); i++)
    {
      cloud.points[i].x = cloud_ros.points[i].x;
      cloud.points[i].y = cloud_ros.points[i].y;
      cloud.points[i].z = cloud_ros.points[i].z;
    }
    return cloud;
  }


  sensor_msgs::PointCloud toROS(const PCLPointCloud& cloud_pcl) {
    sensor_msgs::PointCloud cloud;
    cloud.points.resize(cloud_pcl.points.size());

    for(unsigned int i = 0; i < cloud_pcl.points.size(); i++)
    {
      cloud.points[i].x = cloud_pcl.points[i].x;
      cloud.points[i].y = cloud_pcl.points[i].y;
      cloud.points[i].z = cloud_pcl.points[i].z;
    }
    return cloud;
  }
}
