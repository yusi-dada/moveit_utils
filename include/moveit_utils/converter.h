/**
 * @file converter.h
 * @brief vector配列からROSメッセージ型への変換
 */
#pragma once

#include <moveit_msgs/ObjectColor.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

namespace moveit_utils
{
#define PI (3.14159265359879)
#define DEG2RAD (PI/180.0)
#define RAD2DEG (180.0/PI)

    //************************************************************************
    /**
     * @brief リストからPose型を生成
     * @param [in] list リスト(配列数=3) 位置のみ指定　回転なし
     * @param [in] list リスト(配列数=6) 位置及び回転　回転はオイラー角で指定
     * @param [in] list リスト(配列数=7) 位置及び回転　回転はクォータニオンで指定
     * @return Pose型
     */
    geometry_msgs::Pose list2Pose(std::vector<double> list);

    //************************************************************************
    /**
     * @brief 可変数リストからPose型リストを生成
     * @details 再帰処理用
     * @return Pose型リスト（空）
     */
    std::vector<geometry_msgs::Pose> list2PoseArray();

    //************************************************************************
    /**
     * @brief 可変数リストからPose型リストを生成
     * @param [in] list リスト
     * @param [in] rest 可変リスト（0以上）
     * @return Pose型リスト
     */
    template<class... Rest>
    std::vector<geometry_msgs::Pose> list2PoseArray(std::vector<double> list, Rest&&... rest)
    {
        std::vector<geometry_msgs::Pose> ret = list2PoseArray(rest...);
        ret.insert(ret.begin(),list2Pose(list));
        return ret;
    }    

    //************************************************************************
    /**
     * @brief リストからVector3型を生成
     * @param list リスト(配列数=3)
     * @return Vector3型
     */
    geometry_msgs::Vector3 list2Vec3(std::vector<double> list);

    //************************************************************************
    /**
     * @brief 可変数リストからVector3型リストを生成
     * @details 再帰処理用
     * @return Vector3型リスト（空）
     */
    std::vector<geometry_msgs::Vector3> list2Vec3Array();

    //************************************************************************
    /**
     * @brief 可変数リストからVector3型リストを生成
     * @param [in] list リスト
     * @param [in] rest 可変リスト（0以上）
     * @return Vector3型リスト
     */
    template<class... Rest>
    std::vector<geometry_msgs::Vector3> list2Vec3Array(std::vector<double> list, Rest&&... rest)
    {
        std::vector<geometry_msgs::Vector3> ret = list2Vec3Array(rest...);
        ret.insert(ret.begin(),list2Vec3(list));
        return ret;
    }    

    //************************************************************************
    /**
     * @brief リストからObjectColor型を生成
     * @param list リスト(配列数=4)
     * @return ObjectColor型
     */
    moveit_msgs::ObjectColor list2Color(std::vector<double> gbra);

    //************************************************************************
    /**
     * @brief 可変数リストからObjectColor型リストを生成
     * @return ObjectColor型リスト（空）
     */
    std::vector<moveit_msgs::ObjectColor> list2ColorArray();

    //************************************************************************
    /**
     * @brief 可変数リストからObjectColor型リストを生成
     * @param [in] list リスト
     * @param [in] rest 可変リスト（0以上）
     * @return ObjectColor型リスト
     */
    template<class... Rest>
    std::vector<moveit_msgs::ObjectColor> list2ColorArray(std::vector<double> bgra, Rest&&... rest)
    {
        std::vector<moveit_msgs::ObjectColor> ret = list2ColorArray(rest...);
        ret.insert(ret.begin(),list2Color(bgra));
        return ret;
    }    

}   // namespace moveit_utils