/**
 * @file std_msgs_stream.cpp
 * @brief std_msgsのバイナリ変換
 */
#include "msg2bin/msg2bin.h"

namespace msg2bin
{

#ifdef __ROS_PROGRAM__
// ******************************************************************
//
//	Header
//
// ******************************************************************
std::ofstream& operator << (std::ofstream& stream, const std_msgs::Header dat)
{
	std::vector<int> seq;
	seq.push_back(dat.seq);
	stream << seq;									// seq
	std::vector<int> tim;
	tim.push_back(dat.stamp.sec);
	tim.push_back(dat.stamp.nsec);
	stream << tim;									// stamp
	stream << dat.frame_id;					// frame_id
	return stream;
}
std::ifstream& operator >> (std::ifstream& stream, std_msgs::Header &dat)
{
	std::vector<int> seq;
	stream >> seq;	
	dat.seq = seq[0];
	std::vector<int> tim;
	stream >> tim;
	dat.stamp.sec = tim[0];
	dat.stamp.nsec = tim[1];
	stream >> dat.frame_id;
	return stream;	
}

// ******************************************************************
//
//	Duration
//
// ******************************************************************
std::ofstream& operator << (std::ofstream& stream, const ros::Duration dat)
{
	std::vector<int> dur;
	dur.push_back(dat.sec);
	dur.push_back(dat.nsec);
	stream << dur;
	return stream;
}
std::ifstream& operator >> (std::ifstream& stream, ros::Duration &dat)
{
	std::vector<int> dur;
	stream >> dur;
	dat.sec = dur[0];
	dat.nsec = dur[1];
	return stream;
}
#endif // __ROS_PROGRAM__

// ******************************************************************
//
//	std::vector<int>
//
// ******************************************************************
std::ofstream& operator << (std::ofstream& stream, const std::vector<int> dat)
{
	int count = dat.size();
	stream.write(reinterpret_cast<const char*>(&count), sizeof(count));
	stream.write(reinterpret_cast<const char*>(&dat[0]), dat.size() * sizeof(int));
	return stream;
}
std::ifstream& operator >> (std::ifstream& stream, std::vector<int> &dat)
{
	int count;
	stream.read(reinterpret_cast<char*>(&count), sizeof(count));
	dat.assign(count, 0);
	stream.read(reinterpret_cast<char*>(&dat[0]), dat.size() * sizeof(int));
	return stream;
}

// ******************************************************************
//
//	std::vector<double>
//
// ******************************************************************
std::ofstream& operator << (std::ofstream& stream, const std::vector<double> dat)
{
	int count = dat.size();
	stream.write(reinterpret_cast<const char*>(&count), sizeof(count));
	stream.write(reinterpret_cast<const char*>(&dat[0]), dat.size() * sizeof(double));
	return stream;
}
std::ifstream& operator >> (std::ifstream& stream, std::vector<double> &dat)
{
	int count;
	stream.read(reinterpret_cast<char*>(&count), sizeof(count));
	dat.assign(count, 0);
	stream.read(reinterpret_cast<char*>(&dat[0]), dat.size() * sizeof(double));
	return stream;
}

// ******************************************************************
//
//	std::vector<std::string>
//
// ******************************************************************
std::ofstream& operator << (std::ofstream& stream, const std::vector<std::string> dat)
{
	// データサイズ
	std::vector<int> N;	N.push_back(dat.size());
	stream << N;
	
	for(auto p : dat)
		stream << p;
	return stream;
}
std::ifstream& operator >> (std::ifstream& stream, std::vector<std::string> &dat)
{
	dat.clear();
	std::vector<int> N;	stream >> N;	// データサイズ
	
	for(int i=0; i<N[0]; i++)
	{
		std::string str;
		stream >> str;
		dat.push_back(str);
	}
	return stream;	
}

// ******************************************************************
//
//	std::string
//
// ******************************************************************
std::ofstream& operator << (std::ofstream& stream, const std::string dat)
{
	int count = dat.size();
	stream.write(reinterpret_cast<const char*>(&count), sizeof(count));
	stream.write(reinterpret_cast<const char*>(&dat[0]), dat.size() * sizeof(char));
	return stream;
}
std::ifstream& operator >> (std::ifstream& stream, std::string &dat)
{
	int count;
	stream.read(reinterpret_cast<char*>(&count), sizeof(count));
	dat.assign(count, 0);
	stream.read(reinterpret_cast<char*>(&dat[0]), dat.size() * sizeof(char));
	return stream;
}

}   // namespace msg2bin
