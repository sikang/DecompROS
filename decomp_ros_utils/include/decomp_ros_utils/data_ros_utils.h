#ifndef DECOMP_ROS_UTILS_H
#define DECOMP_ROS_UTILS_H

#include <decomp_geometry/ellipsoid.h>
#include <decomp_geometry/polyhedron.h>
#include <sensor_msgs/PointCloud.h>
#include <decomp_ros_msgs/PolyhedronArray.h>
#include <decomp_ros_msgs/EllipsoidArray.h>
#include <nav_msgs/Path.h>

namespace DecompROS {
inline vec_Vec3f path_to_eigen(const nav_msgs::Path &path) {
  vec_Vec3f vs;
  for (auto it : path.poses) {
    Vec3f v(it.pose.position.x, it.pose.position.y, it.pose.position.z);

    vs.push_back(v);
  }

  return vs;
}

inline nav_msgs::Path eigen_to_path(const vec_Vec3f &vs) {
  nav_msgs::Path path;
  for (auto it : vs) {
    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = it(0);
    pose.pose.position.y = it(1);
    pose.pose.position.z = it(2);
    pose.pose.orientation.w = 1.0;
    pose.pose.orientation.x = 0.0;
    pose.pose.orientation.y = 0.0;
    pose.pose.orientation.z = 0.0;

    path.poses.push_back(pose);
  }

  return path;
}


inline sensor_msgs::PointCloud vec_to_cloud(const vec_Vec3f &pts) {
  sensor_msgs::PointCloud cloud;
  cloud.points.resize(pts.size());

  for (unsigned int i = 0; i < pts.size(); i++) {
    cloud.points[i].x = pts[i](0);
    cloud.points[i].y = pts[i](1);
    cloud.points[i].z = pts[i](2);
  }
  return cloud;
}

inline vec_Vec3f cloud_to_vec(const sensor_msgs::PointCloud &cloud) {
  vec_Vec3f pts;
  pts.resize(cloud.points.size());
  for (unsigned int i = 0; i < cloud.points.size(); i++) {
    pts[i](0) = cloud.points[i].x;
    pts[i](1) = cloud.points[i].y;
    pts[i](2) = cloud.points[i].z;
  }

  return pts;
}

inline Polyhedron3D ros_to_polyhedron(const decomp_ros_msgs::Polyhedron& msg){
  Polyhedron3D poly;
  for(unsigned int i = 0; i < msg.points.size(); i++){
    Vec3f pt(msg.points[i].x,
             msg.points[i].y,
             msg.points[i].z);
    Vec3f n(msg.normals[i].x,
            msg.normals[i].y,
            msg.normals[i].z);
    poly.add(Hyperplane3D(pt, n));
  }
  return poly;
}

inline vec_E<Polyhedron3D> ros_to_polyhedron_array(const decomp_ros_msgs::PolyhedronArray& msg) {
  vec_E<Polyhedron3D> polys(msg.polyhedrons.size());

  for(size_t i = 0; i < msg.polyhedrons.size(); i++)
    polys[i] = ros_to_polyhedron(msg.polyhedrons[i]);

  return polys;
}

inline decomp_ros_msgs::Polyhedron polyhedron_to_ros(const Polyhedron2D& poly){
  decomp_ros_msgs::Polyhedron msg;
  for (const auto &p : poly.hyperplanes()) {
    geometry_msgs::Point pt, n;
    pt.x = p.p_(0);
    pt.y = p.p_(1);
    pt.z = 0;
    n.x = p.n_(0);
    n.y = p.n_(1);
    n.z = 0;
    msg.points.push_back(pt);
    msg.normals.push_back(n);
  }

  return msg;
}

inline decomp_ros_msgs::Polyhedron polyhedron_to_ros(const Polyhedron3D& poly){
  decomp_ros_msgs::Polyhedron msg;
  for (const auto &p : poly.hyperplanes()) {
    geometry_msgs::Point pt, n;
    pt.x = p.p_(0);
    pt.y = p.p_(1);
    pt.z = p.p_(2);
    n.x = p.n_(0);
    n.y = p.n_(1);
    n.z = p.n_(2);
    msg.points.push_back(pt);
    msg.normals.push_back(n);
  }

  return msg;
}


template <int Dim>
decomp_ros_msgs::PolyhedronArray polyhedron_array_to_ros(const vec_E<Polyhedron<Dim>>& vs){
  decomp_ros_msgs::PolyhedronArray msg;
  for (const auto &v : vs)
    msg.polyhedrons.push_back(polyhedron_to_ros(v));
  return msg;
}

inline decomp_ros_msgs::EllipsoidArray ellipsoid_array_to_ros(const vec_E<Ellipsoid3D>& Es) {
  decomp_ros_msgs::EllipsoidArray ellipsoids;
  for (unsigned int i = 0; i < Es.size(); i++) {
    decomp_ros_msgs::Ellipsoid ellipsoid;
    auto d = Es[i].d();
    ellipsoid.d[0] = d(0);
    ellipsoid.d[1] = d(1);
    ellipsoid.d[2] = d(2);

    auto C = Es[i].C();
    for (int x = 0; x < 3; x++)
      for (int y = 0; y < 3; y++)
        ellipsoid.E[3 * x + y] = C(x, y);
    ellipsoids.ellipsoids.push_back(ellipsoid);
  }

  return ellipsoids;
}
}

#endif
