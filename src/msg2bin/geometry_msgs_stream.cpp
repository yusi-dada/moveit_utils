/**
 * @file geometry_msgs_stream.cpp
 * @brief geometry_msgsのバイナリ変換
 */
#include "msg2bin/msg2bin.h"

namespace msg2bin
{

#ifdef __ROS_PROGRAM__
// ******************************************************************
//
//	geometry_msgs::Pose
//
// ******************************************************************
std::ofstream& operator << (std::ofstream& stream, const geometry_msgs::Pose dat)
{
	stream << dat.position;
	stream << dat.orientation;
	return stream;
}
std::ifstream& operator >> (std::ifstream& stream, geometry_msgs::Pose &dat)
{
	stream >> dat.position;
	stream >> dat.orientation;
	return stream;
}

// ******************************************************************
//
//	geometry_msgs::Wrench
//
// ******************************************************************
std::ofstream& operator << (std::ofstream& stream, const geometry_msgs::Wrench dat)
{
	stream << dat.force;
	stream << dat.torque;
	return stream;	
}
std::ifstream& operator >> (std::ifstream& stream, geometry_msgs::Wrench &dat)
{
	stream >> dat.force;
	stream >> dat.torque;
	return stream;	
}

// ******************************************************************
//
//	geometry_msgs::Twist
//
// ******************************************************************
std::ofstream& operator << (std::ofstream& stream, const geometry_msgs::Twist dat)
{
	stream << dat.linear;
	stream << dat.angular;
	return stream;
}
std::ifstream& operator >> (std::ifstream& stream, geometry_msgs::Twist &dat)
{
	stream >> dat.linear;
	stream >> dat.angular;
	return stream;
}

// ******************************************************************
//
//	geometry_msgs::Transform
//
// ******************************************************************
std::ofstream& operator << (std::ofstream& stream, const geometry_msgs::Transform dat)
{
	stream << dat.translation;
	stream << dat.rotation;
	return stream;
}
std::ifstream& operator >> (std::ifstream& stream, geometry_msgs::Transform &dat)
{
	stream >> dat.translation;
	stream >> dat.rotation;
	return stream;
}

// ******************************************************************
//
//	geometry_msgs::Quaternion
//
// ******************************************************************
std::ofstream& operator << (std::ofstream& stream, const geometry_msgs::Quaternion dat)
{
	std::vector<double> vec;
	vec.push_back(dat.x);
	vec.push_back(dat.y);
	vec.push_back(dat.z);
	vec.push_back(dat.w);
	stream << vec;
	return stream;
}
std::ifstream& operator >> (std::ifstream& stream, geometry_msgs::Quaternion &dat)
{
	std::vector<double> vec;
	stream >> vec;
	dat.x = vec[0];
	dat.y = vec[1];
	dat.z = vec[2];
	dat.w = vec[3];
	return stream;
}

// ******************************************************************
//
//	geometry_msgs::Point
//
// ******************************************************************
std::ofstream& operator << (std::ofstream& stream, const geometry_msgs::Point dat)
{
	std::vector<double> vec;
	vec.push_back(dat.x);
	vec.push_back(dat.y);
	vec.push_back(dat.z);
	stream << vec;
	return stream;
}
std::ifstream& operator >> (std::ifstream& stream, geometry_msgs::Point &dat)
{
	std::vector<double> vec;
	stream >> vec;
	dat.x = vec[0];
	dat.y = vec[1];
	dat.z = vec[2];
	return stream;
}

// ******************************************************************
//
//	geometry_msgs::Vector3
//
// ******************************************************************
std::ofstream& operator << (std::ofstream& stream, const geometry_msgs::Vector3 dat)
{
	std::vector<double> vec;
	vec.push_back(dat.x);
	vec.push_back(dat.y);
	vec.push_back(dat.z);
	stream << vec;
	return stream;
}
std::ifstream& operator >> (std::ifstream& stream, geometry_msgs::Vector3 &dat)
{
	std::vector<double> vec;
	stream >> vec;
	dat.x = vec[0];
	dat.y = vec[1];
	dat.z = vec[2];
	return stream;
}

}   // namespace msg2bin

#endif // __ROS_PROGRAM__
