/**
 * @file object_recognition_msgs_stream.cpp
 * @brief object_recognition_msgsのバイナリ変換
 */
#include "msg2bin/msg2bin.h"

namespace msg2bin
{

#ifdef __ROS_PROGRAM__
// ******************************************************************
//
// object_recognition_msgs/ObjectType
//
// ******************************************************************
std::ofstream& operator << (std::ofstream& stream, const object_recognition_msgs::ObjectType dat)
{
	stream << dat.key;
	stream << dat.db;
	return stream;
}
std::ifstream& operator >> (std::ifstream& stream, object_recognition_msgs::ObjectType &dat)
{
	stream >> dat.key;
	stream >> dat.db;
	return stream;
}

}   // namespace msg2bin

#endif // __ROS_PROGRAM__
