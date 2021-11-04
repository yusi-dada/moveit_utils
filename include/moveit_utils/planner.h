/**
 * @file planner.h
 * @brief MoveIt!ドライバ(C++)
 */
#pragma once
#include <map>
#include <string>
#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_interface/planning_interface.h>

#include <moveit_utils/converter.h>

/**
 * @brief MoveIt!ドライバ(C++)
 */
namespace moveit_utils
{

using namespace moveit::planning_interface;

/**
 * @brief MoveIt!ドライバ
 */
class planner
{
    private:
        std::map<std::string, MoveGroupInterfacePtr> move_group;
        PlanningSceneInterface planning_scene_interface;

    private:
        // *************************************************************************
        /**
         * @brief 登録されているかグループ名か判定
         * @param [in] group_name グループ名
         * @retval ture 登録
         * @retval false 未登録 
         */
        bool isValidGroup(std::string group_name);

        // *************************************************************************
        /**
         * @brief 開始関節角を設定
         * @param [in] group_name グループ名
         * @param [in] jnt 関節角[rad]
         * @retval ture 成功
         * @retval false 失敗
         * @note jntに空配列を指定すると現在姿勢を開始姿勢に設定
         */
        bool setStartJoint(std::string group_name, std::vector<double> jnt);

        // *************************************************************************
        /**
         * @brief 目標関節角を設定
         * @param [in] group_name グループ名
         * @param [in] jnt 関節角[rad]
         * @retval ture 成功
         * @retval false 失敗
         */
        bool setGoalJoint(std::string group_name, std::vector<double> jnt);

        // *************************************************************************
        /**
         * @brief プランニングシーン更新まで待機
         * @param [in] obj_name 対象オブジェクト名
         * @param [in] is_known 登録状態
         * @param [in] is_attached 保持状態
         * @param [in] timeout タイムアウト[sec]
         * @retval true 成功
         * @retval false 失敗
         */
        bool waitForUpdate(std::string obj_name, bool is_known, bool is_attached, float timeout=4.0);

    public:
        // *************************************************************************
        /**
         * @brief コンストラクタ
         * @param [in] group_names グループ名リスト
         */
        planner(std::vector<std::string> group_names);

        // *************************************************************************
        /**
         * @brief プランニング
         * @param [in] group_names グループ名リスト
         * @param [in] goal 目標姿勢
         * @param [in] start 初期姿勢
         */
        bool plan(std::string group_name, std::vector<double> goal, std::vector<double> start=std::vector<double>());

        // *************************************************************************
        /**
         * @brief プランニングシーンにBoxオブジェクトを追加
         * @param [in] group_name グループ名
         * @param [in] obj_name 対象オブジェクト名
         * @param [in] Pose オブジェクト姿勢
         * @param [in] Size オブジェクトサイズ
         * @param [in] Color オブジェクト色
         * @retval true 成功
         * @retval false 失敗
         */
        bool addBoxObject(std::string group_name, std::string object_name, std::vector<geometry_msgs::Pose> Pose, std::vector<geometry_msgs::Vector3> Size, std::vector<moveit_msgs::ObjectColor> Color=std::vector<moveit_msgs::ObjectColor>());

        // *************************************************************************
        /**
         * @brief プランニングシーンからオブジェクトを消去
         * @param [in] obj_name 対象オブジェクト名
         * @retval true 成功
         * @retval false 失敗
         */
        bool remObject(std::string object_name);

        // *************************************************************************
        /**
         * @brief 登録グループ名の取得
         * @return 登録グループ名リスト
         */
        std::vector<std::string> getGroupNames();

        // *************************************************************************
        /**
         * @brief 関節角度を取得
         * @param [in] group_name グループ名
         * @return 関節角度リスト[rad]
         * @note グループ名が存在しなければ空リストを返す
         */
        std::vector<double> getCurrentJointValues(std::string group_name);












        // *************************************************************************
        /**
         * @brief プランナーIDを設定
         * @param [in] group_name グループ名
         * @param [in] id プランナーID
         */
        void setPlannerId(std::string group_name, std::string id);

        // *************************************************************************
        /**
         * @brief プランナーIDを取得
         * @param [in] group_name グループ名
         * @return プランナーID
         * @note グループ名が存在しなければ空IDを返す
         */
        std::string getPlannerId(std::string group_name); 

        // *************************************************************************
        /**
         * @brief プランナーパラメータを取得
         * @param [in] group_name グループ名
         * @return プランナーパラメータ
         */
        std::map<std::string, std::string> getPlannerParams (std::string group_name);

        


};


}   // namespace moveit_utils