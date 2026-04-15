#pragma once

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include <kindr/Core>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace kindr_ros {

inline void convertToRosGeometryMsg(const kindr::Position3D& position, geometry_msgs::msg::Point& point) {
  point.x = position.x();
  point.y = position.y();
  point.z = position.z();
}

inline void convertFromRosGeometryMsg(const geometry_msgs::msg::Point& point, kindr::Position3D& position) {
  position = kindr::Position3D(point.x, point.y, point.z);
}

inline void convertToRosGeometryMsg(const kindr::HomTransformQuatD& transform, geometry_msgs::msg::Pose& pose) {
  pose.position.x = transform.getPosition().x();
  pose.position.y = transform.getPosition().y();
  pose.position.z = transform.getPosition().z();

  const Eigen::Quaterniond quaternion(transform.getRotation().matrix());
  pose.orientation = tf2::toMsg(tf2::Quaternion(quaternion.x(), quaternion.y(), quaternion.z(), quaternion.w()));
}

inline void convertFromRosGeometryMsg(const geometry_msgs::msg::Pose& pose, kindr::HomTransformQuatD& transform) {
  const Eigen::Quaterniond quaternion(pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);
  transform.setPosition(kindr::Position3D(pose.position.x, pose.position.y, pose.position.z));
  transform.setRotation(kindr::RotationMatrixD(quaternion.toRotationMatrix()));
}

}  // namespace kindr_ros
