/**
 * @file shape_msgs_stream.cpp
 * @brief shape_msgsのバイナリ変換
 */
#include "msg2bin/msg2bin.h"

namespace msg2bin
{

#ifdef __ROS_PROGRAM__
// ******************************************************************
//
// shape_msgs/SolidPrimitive
//
// ******************************************************************
std::ofstream& operator << (std::ofstream& stream, const shape_msgs::SolidPrimitive dat)
{
	std::vector<int> N(static_cast<int>(dat.type));
	stream << N;
	stream << dat.dimensions;
	return stream;
}
std::ifstream& operator >> (std::ifstream& stream, shape_msgs::SolidPrimitive &dat)
{
	std::vector<int> N;	stream >> N;
	dat.type = static_cast<unsigned char>(N[0]);
	stream >> dat.dimensions;
	return stream;
}

// ******************************************************************
//
// shape_msgs/Mesh
//
// ******************************************************************
std::ofstream& operator << (std::ofstream& stream, const shape_msgs::Mesh dat)
{
	std::vector<int> N1(dat.triangles.size());
	stream << N1;
	for ( auto p : dat.triangles )
		stream << p;
	std::vector<int> N2(dat.vertices.size());
	stream << N2;
	for ( auto p : dat.vertices )
		stream << p;
	return stream;
}
std::ifstream& operator >> (std::ifstream& stream, shape_msgs::Mesh &dat)
{
	std::vector<int> N1;	stream >> N1;
	dat.triangles.resize(N1[0]);
	for( int i=0; i<N1[0]; i++)
		stream >> dat.triangles[i];
	std::vector<int> N2;	stream >> N2;
	dat.vertices.resize(N2[0]);
	for ( int i=0; i<N2[0]; i++ )
		stream >> dat.vertices[i];
	return stream;
}

// ******************************************************************
//
// shape_msgs/MeshTriangle
//
// ******************************************************************
std::ofstream& operator << (std::ofstream& stream, const shape_msgs::MeshTriangle dat)
{
	std::vector<int> N;
	N.push_back(static_cast<int>(dat.vertex_indices[0]));
	N.push_back(static_cast<int>(dat.vertex_indices[1]));
	N.push_back(static_cast<int>(dat.vertex_indices[2]));
	stream << N;
	return stream;
}
std::ifstream& operator >> (std::ifstream& stream, shape_msgs::MeshTriangle &dat)
{
	std::vector<int> N;	stream >> N;
	for(int i=0; i<3; i++)
		dat.vertex_indices[i] = static_cast<unsigned int>(N[i]);
	return stream;
}

// ******************************************************************
//
// shape_msgs/Plane
//
// ******************************************************************
std::ofstream& operator << (std::ofstream& stream, const shape_msgs::Plane dat)
{
	std::vector<double> N;
	N.push_back(dat.coef[0]);
	N.push_back(dat.coef[1]);
	N.push_back(dat.coef[2]);
	N.push_back(dat.coef[3]);
	stream << N;
	return stream;
}
std::ifstream& operator >> (std::ifstream& stream, shape_msgs::Plane &dat)
{
	std::vector<double> N;	stream >> N;
	for(int i=0; i<4; i++)
		dat.coef[i] = N[i];
	return stream;
}

}   // namespace msg2bin

#endif // __ROS_PROGRAM__

