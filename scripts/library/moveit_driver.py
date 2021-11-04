# -*- coding: utf-8 -*-
##
# @file moveit_driver.py
# @brief MoveIt!ドライバ（python）
import sys
import copy
from math import pi
import moveit_msgs

import rospy
import geometry_msgs.msg
import moveit_commander
from moveit_commander.conversions import *
from moveit_msgs.msg import (
    TrajectoryConstraints,
    OrientationConstraint,
    Constraints
)

##
# @brief MoveIt!ドライバ(python)
class moveit_driver(object):

    ## *********************************************************************************
    # @brief コンストラクタ
    def __init__(self):
        super(moveit_driver, self).__init__()

        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('moveit_driver', anonymous=True)

        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface(synchronous=True)
        
        group_names = self.robot.get_group_names()
        self.move_group = {}
        for name in group_names:
            self.move_group[name] = moveit_commander.MoveGroupCommander(name)

            self.move_group[name].set_planner_id('manipulator[BiTRRT]')
            self.move_group[name].set_max_velocity_scaling_factor(1.0)
            self.move_group[name].set_max_acceleration_scaling_factor(1.0)

            #print(self.robot.get_robot_markers(name))

    ## *********************************************************************************
    # @brief グループ毎の関節角[deg]を表示
    def status(self):
        group = self.move_group.keys()
        for i, g in enumerate(group):
            s = ["{:5.3f}".format(j*180.0/pi) for j in self.getJoint(g)]
            s = "[" + ", ".join(s) + "]"
            rospy.loginfo("group-{0} : {1} {2}".format(i, g, s))

    ## *********************************************************************************
    # @brief グループの現在の関節角を取得
    # @param name (str)グループ名
    # @retval None グループ名が存在しない場合
    # @retval joint 現在の関節角（リスト）
    # @note グループ名リストはrobot.get_group_names()で取得可能
    def getJoint(self, name):
        if name in self.move_group:
            joint = self.move_group[name].get_current_joint_values()
            return joint
        else:
            rospy.logerr("[getJoint] {0} is not group name.".format(name))
            return None

    ## *********************************************************************************
    # @brief 軌道計画し実行
    # @param name (str) グループ名
    # @param goal (list of floats / Pose) 目標関節角[rad]
    # @return 動作結果
    def plan_and_move(self, name, goal):
        ret = self.plan(name, goal)
        if ret == None:
            rospy.logerr("[plan_and_move] planning failed")
            return False
        else:
            return self.move_group[name].execute(ret)

    ## *********************************************************************************
    # @brief 軌道計画を実施
    # @param name (str) グループ名
    # @param goal (list of floats / Pose) 目標関節角[rad]
    # @param start (list of floats) 初期関節角[rad]
    # @retval trajectory (RobotTrajectory) 計画結果
    # @retval None 失敗
    def plan(self, name, goal, start=None):

        if name in self.move_group:
            robot_state = self.move_group[name].get_current_state()
            joint_size = len(robot_state.joint_state.position)

            if not type(goal) is geometry_msgs.msg.Pose:
                if joint_size != len(goal):
                    rospy.logerr("[plan] goal size must be {0}".format(joint_size))
                    return None
            
            if (start != None) and (joint_size != len(start)):
                rospy.logerr("[plan] start size must be {0}".format(joint_size))
                return None

            if start == None:
                self.move_group[name].set_start_state_to_current_state()
            else:
                robot_state.joint_state.position = start
                self.move_group[name].set_start_state(robot_state)

            if type(goal) is geometry_msgs.msg.Pose:
                goal_state = goal
            else:
                goal_state = robot_state.joint_state
                goal_state.position = goal

            """
            oconst = OrientationConstraint()
            q = tf.transformations.quaternion_from_euler(pi,0,0)
            oconst.orientation.x = q[0]
            oconst.orientation.y = q[1]
            oconst.orientation.z = q[2]
            oconst.orientation.w = q[3]
            oconst.link_name = self.move_group[name].get_end_effector_link()
            const = Constraints()
            const.orientation_constraints.append(oconst)
            tconst = TrajectoryConstraints()
            tconst.constraints.append(const)
            self.move_group[name].set_trajectory_constraints(tconst)
            """

            success, trajectory, planning_time, error_code = self.move_group[name].plan(goal_state)

            if success:
                rospy.loginfo("[plan] success. planning_time={0}[sec]".format(planning_time))
                trajectory = self.move_group[name].retime_trajectory(self.move_group[name].get_current_state(), trajectory)
                return trajectory
            else:
                rospy.logerr("[plan] planning failed. {0}".format(error_code))
                return None

        else:
            rospy.logerr("[plan] {0} is not group name.".format(name))
            return None
    
    ## *********************************************************************************
    # @brief ワークスペースオブジェクトを追加
    # @param ws (str) ワークスペース
    def set_workspace(self, ws, obj_name=""):
        self.add_object("BOX", "{0}-Z".format(obj_name), [(ws[3]+ws[0])/2, (ws[4]+ws[1])/2, ws[2], 0,0,0], (ws[3]-ws[0],ws[4]-ws[1],0.001))
        self.add_object("BOX", "{0}+Z".format(obj_name), [(ws[3]+ws[0])/2, (ws[4]+ws[1])/2, ws[5], 0,0,0], (ws[3]-ws[0],ws[4]-ws[1],0.001))
        self.add_object("BOX", "{0}-X".format(obj_name), [ws[0], (ws[4]+ws[1])/2, (ws[5]+ws[2])/2, 0,0,0], (0.001, ws[4]-ws[1],ws[5]-ws[2]))
        self.add_object("BOX", "{0}+X".format(obj_name), [ws[3], (ws[4]+ws[1])/2, (ws[5]+ws[2])/2, 0,0,0], (0.001, ws[4]-ws[1],ws[5]-ws[2]))
        self.add_object("BOX", "{0}-Y".format(obj_name), [(ws[3]+ws[0])/2, ws[1], (ws[5]+ws[2])/2, 0,0,0], (ws[3]-ws[0],0.001,ws[5]-ws[2]))
        self.add_object("BOX", "{0}+Y".format(obj_name), [(ws[3]+ws[0])/2, ws[4], (ws[5]+ws[2])/2, 0,0,0], (ws[3]-ws[0],0.001,ws[5]-ws[2]))

    ## *********************************************************************************
    # @brief プランニングシーンにオブジェクトを追加
    # @param obj_type (str) オブジェクト型名
    # @param name (str) オブジェクト名
    # @param pose (list of floats / Pose / PoseStamped) オブジェクト姿勢
    # @param arg1 追加パラメータ1
    # @param arg2 追加パラメータ2
    # @param timeout (float) 処理タイムアウト[sec]
    # @retval True 成功
    # @retval False 失敗
    def add_object(self, obj_type, name, pose, arg1=None, arg2=None, timeout=4):

        frame = self.robot.get_planning_frame()
        if type(pose) is geometry_msgs.msg.PoseStamped:
            pose_ = pose
            pose_.header.frame_id = frame
        elif type(pose) is geometry_msgs.msg.Pose:
            pose_ = geometry_msgs.msg.PoseStamped()
            pose_.pose = pose
            pose_.header.frame_id = frame       
        elif type(pose) is list:
            pose_ = list_to_pose_stamped(pose, frame)
        else:
            rospy.logerr("[add_object] Wrong pose type")
            return False

        try:
            if obj_type == "SPHERE":
                self.scene.add_sphere(name, pose_, radius=arg1)
            elif obj_type == "CYLINDER":
                self.scene.add_cylinder(name, pose_, height=arg1, radius=arg2)
            elif obj_type == "MESH":
                self.scene.add_mesh(name, pose_, filename=arg1, size=arg2)
            elif obj_type == "BOX":
                self.scene.add_box(name, pose_, size=arg1)
            elif obj_type == "PLANE":
                self.scene.add_plane(name, pose_, normal=arg1, offset=arg2)
            else:
                rospy.logerr("[add_object] Wrong object type name")
                return False

        except Exception as e:
            rospy.logerr("[add_object] exception of %s", e)
            return False

        return self.wait_for_state_update(name, is_known=True, timeout=timeout)

    ## *********************************************************************************
    # @brief プランニングシーンからオブジェクトを消去
    # @param name (str) オブジェクト名
    # @param timeout (float) 処理タイムアウト[sec]
    # @retval True 成功
    # @retval False 失敗
    def rem_object(self, name=None, timeout=4):
        if name is None:
            for obj in self.scene.get_known_object_names():
                self.scene.remove_world_object(obj)
                if not self.wait_for_state_update(obj, timeout=timeout):
                    return False
            return True
        if name in self.scene.get_known_object_names():
            self.scene.remove_world_object(name)
            return self.wait_for_state_update(name, timeout=timeout)
        rospy.logwarn("[rem_object] {0} is unregistered.")
        return True

    ## *********************************************************************************
    # @brief プランニングシーンのオブジェクトをロボットに取り付け
    # @param object_name (str) オブジェクト名
    # @param group_name (str) グループ名
    # @param link_name (str) 取り付けリンク名
    # @param timeout (float) 処理タイムアウト[sec]
    # @retval True 成功
    # @retval False 失敗
    def attach_object(self, object_name, group_name, link_name=None, timeout=4):

        if not group_name in self.move_group:
            rospy.logerr("{0} is not group name.".format(group_name))
            return False
        if not object_name in self.scene.get_known_object_names():
            rospy.logerr("{0} is not object name.".format(object_name))
            return False
        
        if link_name == None:
            link_name = self.move_group[group_name].get_end_effector_link()
        self.move_group[group_name].attach_object(object_name, link_name, touch_links=[])

        return self.wait_for_state_update(object_name, is_attached=True, timeout=timeout)

    ## *********************************************************************************
    # @brief オブジェクトをロボットから取り外し
    # @param object_name (str) オブジェクト名
    # @param group_name (str) グループ名
    # @param link_name (str) 取り付けリンク名
    # @param timeout (float) 処理タイムアウト[sec]
    # @retval True 成功
    # @retval False 失敗
    def detach_object(self, object_name, group_name, timeout=4):

        if not group_name in self.move_group:
            rospy.logerr("{0} is not group name.".format(group_name))
            return False

        attached_objects = self.scene.get_attached_objects([object_name])
        if len(attached_objects.keys()) == 0:
            rospy.logerr("{0} is not attached.".format(object_name))
            return False

        self.move_group[group_name].detach_object(object_name)

        return self.wait_for_state_update(object_name, is_known=True, timeout=timeout)

    ## *********************************************************************************
    # @brief プランニングシーン内のオブジェクト状態が更新されるまで待機
    # @param name (str) 対象オブジェクト
    # @param is_known (bool) プランニングシーン内に登録済
    # @param is_attached (bool) ロボットに接続されているか否か
    # @param timeout (double) タイムアウト時間[sec]
    # @retval True 成功
    # @retval False 失敗
    def wait_for_state_update(self, name, is_known=False, is_attached=False, timeout=4):
        
        start = rospy.get_time()
        seconds = rospy.get_time()
        while (seconds - start < timeout) and not rospy.is_shutdown():
            attached_objects = self.scene.get_attached_objects([name])
            is_attached_ = len(attached_objects.keys()) > 0
            is_known_    = name in self.scene.get_known_object_names()

            if (is_attached == is_attached_) and (is_known == is_known_):
                return True

            rospy.sleep(0.1)
            seconds = rospy.get_time()

        return False