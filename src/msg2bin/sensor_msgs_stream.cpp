/**
 * @file sensor_msgs_stream.cpp
 * @brief sensor_msgsのバイナリ変換
 */
#include "msg2bin/msg2bin.h"

namespace msg2bin
{

#ifdef __ROS_PROGRAM__
// ******************************************************************
//
// sensor_msgs/JointState
//
// ******************************************************************
std::ofstream& operator << (std::ofstream& stream, const sensor_msgs::JointState dat)
{
	stream << dat.header;
	stream << dat.name;
	stream << dat.position;
	stream << dat.velocity;
	stream << dat.effort;
	return stream;
}
std::ifstream& operator >> (std::ifstream& stream, sensor_msgs::JointState &dat)
{
	stream >> dat.header;
	stream >> dat.name;
	stream >> dat.position;
	stream >> dat.velocity;
	stream >> dat.effort;
	return stream;
}

// ******************************************************************
//
// sensor_msgs/MultiDOFJointState
//
// ******************************************************************
std::ofstream& operator << (std::ofstream& stream, const sensor_msgs::MultiDOFJointState dat)
{
	stream << dat.header;
	stream << dat.joint_names;
	std::vector<int> N1;	N1.push_back(dat.transforms.size());
	std::vector<int> N2;	N2.push_back(dat.twist.size());
	std::vector<int> N3;	N3.push_back(dat.wrench.size());
	
	stream << N1;
	for(auto p : dat.transforms)
		stream << p;

	stream << N2;
	for(auto p : dat.twist)
		stream << p;

	stream << N2;
	for(auto p : dat.wrench)
		stream << p;
	
	return stream;	
}
std::ifstream& operator >> (std::ifstream& stream, sensor_msgs::MultiDOFJointState &dat)
{
	stream >> dat.header;
	stream >> dat.joint_names;

	std::vector<int> N1,N2,N3;
	
	stream >> N1;
	dat.transforms.resize(N1[0]);
	for(int i=0; i<N1[0]; i++)
		stream >> dat.transforms[i];

	stream >> N2;
	dat.twist.resize(N2[0]);
	for(int i=0; i<N2[0]; i++)
		stream >> dat.twist[i];

	stream >> N3;
	dat.wrench.resize(N3[0]);
	for(int i=0; i<N3[0]; i++)
		stream >> dat.wrench[i];
	
	return stream;
}

}   // namespace msg2bin

#endif // __ROS_PROGRAM__
