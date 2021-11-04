/**
 * @file trajectory_msgs_stream.cpp
 * @brief trajectory_msgsのバイナリ変換
 */
#include "msg2bin/msg2bin.h"

namespace msg2bin
{

#ifdef __ROS_PROGRAM__
// ******************************************************************
//
//	trajectory_msgs::JointTrajectory
//
// ******************************************************************
std::ofstream& operator << (std::ofstream& stream, const trajectory_msgs::JointTrajectory dat)
{
	stream << dat.header;
	stream << dat.joint_names;
	std::vector<int> N;	N.push_back(dat.points.size());
	stream << N;	// データ数
	for(int i=0; i<N[0]; i++)
		stream << dat.points[i];
	return stream;
}
std::ifstream& operator >> (std::ifstream& stream, trajectory_msgs::JointTrajectory &dat)
{
	stream >> dat.header;
	stream >> dat.joint_names;
	std::vector<int> N;
	stream >> N;	// データ数
	dat.points.resize(N[0]);
	for(int i=0; i<N[0]; i++)
		stream >> dat.points[i];
	return stream;
}

// ******************************************************************
//
//	trajectory_msgs::JointTrajectoryPoint
//
// ******************************************************************
std::ofstream& operator << (std::ofstream& stream, const trajectory_msgs::JointTrajectoryPoint dat)
{
	stream << dat.positions;
	stream << dat.velocities;
	stream << dat.accelerations;
	stream << dat.effort;
	stream << dat.time_from_start;
	return stream;
}
std::ifstream& operator >> (std::ifstream& stream, trajectory_msgs::JointTrajectoryPoint &dat)
{
	stream >> dat.positions;
	stream >> dat.velocities;
	stream >> dat.accelerations;
	stream >> dat.effort;
	stream >> dat.time_from_start;
	return stream;
}

// ******************************************************************
//
//	trajectory_msgs/MultiDOFJointTrajectory
//
// ******************************************************************
std::ofstream& operator << (std::ofstream& stream, const trajectory_msgs::MultiDOFJointTrajectory dat)
{
	stream << dat.header;
	stream << dat.joint_names;

	std::vector<int> N;
	N.push_back(dat.points.size());
	stream << N;

	for(auto p : dat.points)
		stream << p;

	return stream;	
}
std::ifstream& operator >> (std::ifstream& stream, trajectory_msgs::MultiDOFJointTrajectory &dat)
{
	stream >> dat.header;
	stream >> dat.joint_names;

	std::vector<int> N;	stream >> N;

	dat.points.resize(N[0]);
	for(int i=0; i<N[0]; i++)
		stream >> dat.points[i];

	return stream;	
}

// ******************************************************************
//
//	trajectory_msgs::MultiDOFJointTrajectoryPoint
//
// ******************************************************************
std::ofstream& operator << (std::ofstream& stream, const trajectory_msgs::MultiDOFJointTrajectoryPoint dat)
{
	std::vector<int> N1; N1.push_back(dat.transforms.size());
	std::vector<int> N2; N2.push_back(dat.velocities.size());
	std::vector<int> N3; N3.push_back(dat.accelerations.size());
	
	stream << N1;
	for(auto p : dat.transforms)		{stream << p;}	// transform[]
	stream << N2;
	for(auto p : dat.velocities)		{stream << p;}	// velocities[]
	stream << N3;
	for(auto p : dat.accelerations)	{stream << p;}	// accelerations[]
	stream << dat.time_from_start;									// time_from_start
	return stream;
}
std::ifstream& operator >> (std::ifstream& stream, trajectory_msgs::MultiDOFJointTrajectoryPoint &dat)
{
	std::vector<int> N1,N2,N3;

	stream >> N1;
	dat.transforms.resize(N1[0]);
	for (int i=0; i<N1[0]; i++)
		stream >> dat.transforms[i];
	stream >> N2;
	dat.velocities.resize(N2[0]);
	for (int i=0; i<N2[0]; i++)
		stream >> dat.velocities[i];
	stream >> N3;
	dat.accelerations.resize(N3[0]);
	for (int i=0; i<N3[0]; i++)
		stream >> dat.accelerations[i];
	stream >> dat.time_from_start;
	return stream;
}
#endif // __ROS_PROGRAM__

}   // namespace msg2bin

