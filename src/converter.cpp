/**
 * @file converter.cpp
 * @brief vector配列からROSメッセージ型への変換
 */
#include <moveit_utils/converter.h>
#include <cassert>

namespace moveit_utils
{
    //*******************************************************************
    std::vector<geometry_msgs::Pose> list2PoseArray()
    {
        return std::vector<geometry_msgs::Pose>();
    }
    //*******************************************************************
    std::vector<geometry_msgs::Vector3> list2Vec3Array()
    {
        return std::vector<geometry_msgs::Vector3>();
    }
    //*******************************************************************
    std::vector<moveit_msgs::ObjectColor> list2ColorArray()
    {
        return std::vector<moveit_msgs::ObjectColor>();        
    }

    //*******************************************************************
    geometry_msgs::Pose list2Pose(std::vector<double> list)
    {
        assert( (list.size()==3) || (list.size()==6) || (list.size()==7));
        geometry_msgs::Pose pose;
        switch(list.size())
        {
            case 3: // only position
                pose.position.x  = list[0];
                pose.position.y  = list[1];
                pose.position.z  = list[2];
                pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0.0, 0.0, 0.0);
                return pose;
            case 6: // position + Euler angle
                pose.position.x  = list[0];
                pose.position.y  = list[1];
                pose.position.z  = list[2];
                pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(list[3],list[4],list[5]);
                return pose;
            case 7: // position + Quaternion
                pose.position.x  = list[0];
                pose.position.y  = list[1];
                pose.position.z  = list[2];
                pose.orientation.x = list[3];
                pose.orientation.y = list[4];
                pose.orientation.z = list[5];
                pose.orientation.w = list[6];
                return pose;
        }

    }

    //*******************************************************************
    geometry_msgs::Vector3 list2Vec3(std::vector<double> list)
    {
        assert(list.size()==3);
        geometry_msgs::Vector3 Vec3;
        Vec3.x = list[0];
        Vec3.y = list[1];
        Vec3.z = list[2];
        return Vec3;
    }

    //*******************************************************************
    moveit_msgs::ObjectColor list2Color(std::vector<double> bgra)
    {
        assert(bgra.size()==4);
        moveit_msgs::ObjectColor color;
        color.color.b = bgra[0];
        color.color.g = bgra[1];
        color.color.r = bgra[2];
        color.color.a = bgra[3];
        return color;
    }

}   // namespace moveit_utils