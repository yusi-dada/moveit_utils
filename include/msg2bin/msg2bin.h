/**
 * @file msg2bin.h
 * @brief ROSメッセージ型のバイナリ変換
 */
#pragma once
#define __ROS_PROGRAM__
#include <fstream>
#include <vector>

#ifdef __ROS_PROGRAM__
    #include <ros/ros.h>
    #include <ros/console.h>
    #include <moveit_msgs/DisplayTrajectory.h>
#endif

/**
 * @brief ROSメッセージ型のバイナリ変換
 */
namespace msg2bin
{

//******************************************************************************************
//
// moveit_msgs
//
//******************************************************************************************
/**
 * @brief Output DisplayTrajectory to ofstream
 */
std::ofstream& operator << (std::ofstream& stream, const moveit_msgs::DisplayTrajectory dat);

/**
 * @brief Input DisplayTrajectory from ifstream
 */
std::ifstream& operator >> (std::ifstream& stream, moveit_msgs::DisplayTrajectory &dat);

/**
 * @brief Output RobotState to ofstream
 */
std::ofstream& operator << (std::ofstream& stream, const moveit_msgs::RobotState dat);

/**
 * @brief Input RobotState from ifstream
 */
std::ifstream& operator >> (std::ifstream& stream, moveit_msgs::RobotState &dat);

/**
 * @brief Output RobotTrajectory to ofstream
 */
std::ofstream& operator << (std::ofstream& stream, const moveit_msgs::RobotTrajectory dat);

/**
 * @brief Input RobotTrajectory from ifstream
 */
std::ifstream& operator >> (std::ifstream& stream, moveit_msgs::RobotTrajectory &dat);

/**
 * @brief Output AttachedCollisionObject to ofstream
 */
std::ofstream& operator << (std::ofstream& stream, const moveit_msgs::AttachedCollisionObject dat);

/**
 * @brief Input AttachedCollisionObject from ifstream
 */
std::ifstream& operator >> (std::ifstream& stream, moveit_msgs::AttachedCollisionObject &dat);

/**
 * @brief Output CollisionObject to ofstream
 */
std::ofstream& operator << (std::ofstream& stream, const moveit_msgs::CollisionObject dat);

/**
 * @brief Input CollisionObject from ifstream
 */
std::ifstream& operator >> (std::ifstream& stream, moveit_msgs::CollisionObject &dat);


//******************************************************************************************
//
// trajectory_msgs
//
//******************************************************************************************
/**
 * @brief Output JointTrajectory to ofstream
 */
std::ofstream& operator << (std::ofstream& stream, const trajectory_msgs::JointTrajectory dat);

/**
 * @brief Input JointTrajectory from ifstream
 */
std::ifstream& operator >> (std::ifstream& stream, trajectory_msgs::JointTrajectory &dat);

/**
 * @brief Output JointTrajectoryPoint to ofstream
 */
std::ofstream& operator << (std::ofstream& stream, const trajectory_msgs::JointTrajectoryPoint dat);

/**
 * @brief Input JointTrajectoryPoint from ifstream
 */
std::ifstream& operator >> (std::ifstream& stream, trajectory_msgs::JointTrajectoryPoint &dat);

/**
 * @brief Output MultiDOFJointTrajectory to ofstream
 */
std::ofstream& operator << (std::ofstream& stream, const trajectory_msgs::MultiDOFJointTrajectory dat);

/**
 * @brief Input MultiDOFJointTrajectory from ifstream
 */
std::ifstream& operator >> (std::ifstream& stream, trajectory_msgs::MultiDOFJointTrajectory &dat);

/**
 * @brief Output MultiDOFJointTrajectoryPoint to ofstream
 */
std::ofstream& operator << (std::ofstream& stream, const trajectory_msgs::MultiDOFJointTrajectoryPoint dat);

/**
 * @brief Input MultiDOFJointTrajectoryPoint from ifstream
 */
std::ifstream& operator >> (std::ifstream& stream, trajectory_msgs::MultiDOFJointTrajectoryPoint &dat);

//******************************************************************************************
//
// shape_msgs
//
//******************************************************************************************
/**
 * @brief Output SolidPrimitive to ofstream
 */
std::ofstream& operator << (std::ofstream& stream, const shape_msgs::SolidPrimitive dat);

/**
 * @brief Input SolidPrimitive from ifstream
 */
std::ifstream& operator >> (std::ifstream& stream, shape_msgs::SolidPrimitive &dat);

/**
 * @brief Output Mesh to ofstream
 */
std::ofstream& operator << (std::ofstream& stream, const shape_msgs::Mesh dat);

/**
 * @brief Input Mesh from ifstream
 */
std::ifstream& operator >> (std::ifstream& stream, shape_msgs::Mesh &dat);

/**
 * @brief Output MeshTriangle to ofstream
 */
std::ofstream& operator << (std::ofstream& stream, const shape_msgs::MeshTriangle dat);

/**
 * @brief Input MeshTriangle from ifstream
 */
std::ifstream& operator >> (std::ifstream& stream, shape_msgs::MeshTriangle &dat);

/**
 * @brief Output Plane to ofstream
 */
std::ofstream& operator << (std::ofstream& stream, const shape_msgs::Plane dat);

/**
 * @brief Input Plane from ifstream
 */
std::ifstream& operator >> (std::ifstream& stream, shape_msgs::Plane &dat);

//******************************************************************************************
//
// sensor_msgs
//
//******************************************************************************************
/**
 * @brief Output JointState to ofstream
 */
std::ofstream& operator << (std::ofstream& stream, const sensor_msgs::JointState dat);

/**
 * @brief Input JointState from ifstream
 */
std::ifstream& operator >> (std::ifstream& stream, sensor_msgs::JointState &dat);

/**
 * @brief Output MultiDOFJointState to ofstream
 */
std::ofstream& operator << (std::ofstream& stream, const sensor_msgs::MultiDOFJointState dat);

/**
 * @brief Input MultiDOFJointState from ifstream
 */
std::ifstream& operator >> (std::ifstream& stream, sensor_msgs::MultiDOFJointState &dat);

//******************************************************************************************
//
// geometry_msgs
//
//******************************************************************************************
/**
 * @brief Output Pose to ofstream
 */
std::ofstream& operator << (std::ofstream& stream, const geometry_msgs::Pose dat);

/**
 * @brief Input Pose from ifstream
 */
std::ifstream& operator >> (std::ifstream& stream, geometry_msgs::Pose &dat);

/**
 * @brief Output Wrench to ofstream
 */
std::ofstream& operator << (std::ofstream& stream, const geometry_msgs::Wrench dat);

/**
 * @brief Input Wrench from ifstream
 */
std::ifstream& operator >> (std::ifstream& stream, geometry_msgs::Wrench &dat);

/**
 * @brief Output Twist to ofstream
 */
std::ofstream& operator << (std::ofstream& stream, const geometry_msgs::Twist dat);

/**
 * @brief Input Twist from ifstream
 */
std::ifstream& operator >> (std::ifstream& stream, geometry_msgs::Twist &dat);

/**
 * @brief Output Transform to ofstream
 */
std::ofstream& operator << (std::ofstream& stream, const geometry_msgs::Transform dat);

/**
 * @brief Input Transform from ifstream
 */
std::ifstream& operator >> (std::ifstream& stream, geometry_msgs::Transform &dat);

/**
 * @brief Output Quaternion to ofstream
 */
std::ofstream& operator << (std::ofstream& stream, const geometry_msgs::Quaternion dat);

/**
 * @brief Input Quaternion from ifstream
 */
std::ifstream& operator >> (std::ifstream& stream, geometry_msgs::Quaternion &dat);

/**
 * @brief Output Point to ofstream
 */
std::ofstream& operator << (std::ofstream& stream, const geometry_msgs::Point dat);

/**
 * @brief Input Point from ifstream
 */
std::ifstream& operator >> (std::ifstream& stream, geometry_msgs::Point &dat);

/**
 * @brief Output Vector3 to ofstream
 */
std::ofstream& operator << (std::ofstream& stream, const geometry_msgs::Vector3 dat);

/**
 * @brief Input Vector3 from ifstream
 */
std::ifstream& operator >> (std::ifstream& stream, geometry_msgs::Vector3 &dat);

//******************************************************************************************
//
// object_recognition_msgs
//
//******************************************************************************************
/**
 * @brief Output ObjectType to ofstream
 */
std::ofstream& operator << (std::ofstream& stream, const object_recognition_msgs::ObjectType dat);

/**
 * @brief Input ObjectType from ifstream
 */
std::ifstream& operator >> (std::ifstream& stream, object_recognition_msgs::ObjectType &dat);

//******************************************************************************************
//
// std_msgs
//
//******************************************************************************************
/**
 * @brief Output Header to ofstream
 */
std::ofstream& operator << (std::ofstream& stream, const std_msgs::Header dat);

/**
 * @brief Input Header from ifstream
 */
std::ifstream& operator >> (std::ifstream& stream, std_msgs::Header &dat);

/**
 * @brief Output Duration to ofstream
 */
std::ofstream& operator << (std::ofstream& stream, const ros::Duration dat);

/**
 * @brief Input Duration from ifstream
 */
std::ifstream& operator >> (std::ifstream& stream, ros::Duration &dat);

/**
 * @brief Output vector<int> to ofstream
 */
std::ofstream& operator << (std::ofstream& stream, const std::vector<int> dat);

/**
 * @brief Input vector<int> from ifstream
 */
std::ifstream& operator >> (std::ifstream& stream, std::vector<int> &dat);

/**
 * @brief Output vector<double> to ofstream
 */
std::ofstream& operator << (std::ofstream& stream, const std::vector<double> dat);

/**
 * @brief Input vector<double> from ifstream
 */
std::ifstream& operator >> (std::ifstream& stream, std::vector<double> &dat);

/**
 * @brief Output vector<string> to ofstream
 */
std::ofstream& operator << (std::ofstream& stream, const std::vector<std::string> dat);

/**
 * @brief Input vector<string> from ifstream
 */
std::ifstream& operator >> (std::ifstream& stream, std::vector<std::string> &dat);

/**
 * @brief Output string to ofstream
 */
std::ofstream& operator << (std::ofstream& stream, const std::string dat);

/**
 * @brief Input string from ifstream
 */
std::ifstream& operator >> (std::ifstream& stream, std::string &dat);

}   // namespace msg2bin
