#!/usr/bin/python3
# -*- coding: utf-8 -*-

from library.moveit_driver import *
import urdf_parser_py

import sys
import argparse

from urdf_parser_py.urdf import URDF, Joint

if __name__ == '__main__':

    # ドライバ生成・状態表示
    group = "manipulator"
    md = moveit_driver()
    md.status()
    
    ws = [-0.65, -0.63, 0.05, 0.35, 0.38, 1.0]
    md.set_workspace(ws, "test")

