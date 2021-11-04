#!/usr/bin/python3
# -*- coding: utf-8 -*-

from library.moveit_driver import *
import urdf_parser_py

import sys
import argparse

from urdf_parser_py.urdf import URDF, Joint

if __name__ == '__main__':

    # ドライバ生成・状態表示
    md = moveit_driver()
    md.status()

    # プランナー表示
    group = "manipulator"
    des = md.move_group[group].get_interface_description()
    print(des.planner_ids)

    # 周辺環境設定 
    md.add_object('BOX', 'box', [0.3, 0.3, 0.3, 0,0,0], (0.1,0.1,0.1))
    md.add_object('BOX', 'box1', [-0.3, -0.3, 0.3, 0,0,0], (0.1,0.1,0.1))
    md.add_object('BOX', 'box2', [-0.1, -0.3, 0.4, 0,0,0], (0.01,0.4,0.3))
    md.add_object('BOX', 'box3', [-0.3, -0.1, 0.4, 0,0,0], (0.4,0.01,0.3))

    # オブジェクトの姿勢を取得    
    obj_pose = md.scene.get_object_poses(['box','box1'])
    p = obj_pose['box']
    pose = [p.position.x,p.position.y,p.position.z+0.05, 3.14, 0, 0]
    p = obj_pose['box1']
    pose1 = [p.position.x,p.position.y,p.position.z+0.15, 3.14, 0, 0]

    md.plan_and_move(group, list_to_pose(pose))
    md.attach_object('box',group)

    md.plan_and_move(group, list_to_pose(pose1))
    md.detach_object('box',group)

    md.plan_and_move(group, list_to_pose(pose))

    md.rem_object()

    """
    group = "robot4"


    j1 = [-57.250, 53.765, 53.644, -2.181, 66.149, 218.148, -38.079, 54.991, 115.613, 1.882, 13.980, 139.443]
    j2 = [ 57.250, 53.765, 53.644, -2.181, 66.149, 218.148,  38.079, 54.991, 115.613, 1.882, 13.980, 139.443]
    md.plan_and_move("robot",[j*pi/180 for j in j1])
    md.plan_and_move("robot",[j*pi/180 for j in j2])



    md.add_object('BOX', 'box', [0.3, 0.3, 0.3, 0,0,0], (0.1,0.1,0.1))
    md.add_object('BOX', 'box1', [-0.3, -0.3, 0.3, 0,0,0], (0.1,0.1,0.1))
    md.add_object('BOX', 'box2', [-0.1, -0.3, 0.4, 0,0,0], (0.01,0.4,0.3))
    md.add_object('BOX', 'box3', [-0.3, -0.1, 0.4, 0,0,0], (0.4,0.01,0.3))

    obj_pose = md.scene.get_object_poses(['box','box1'])
    p = obj_pose['box']
    pose = [p.position.x,p.position.y,p.position.z+0.05, 3.14, 0, 0]
    p = obj_pose['box1']
    pose1 = [p.position.x,p.position.y,p.position.z+0.15, 3.14, 0, 0]

    md.plan_and_move(group, list_to_pose(pose))
    md.attach_object('box',group)

    md.plan_and_move(group, list_to_pose(pose1))
    md.detach_object('box',group)

    md.plan_and_move(group, list_to_pose(pose))

    md.rem_object('box')
    md.rem_object('box1')
    md.rem_object('box2')
    md.rem_object('box3')

    #md.plan("manipulator", [0,0,0,0,0,0])

    print(len("test"))

    rospy.spin()

    """
