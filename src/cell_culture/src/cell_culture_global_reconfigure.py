#!/usr/bin/env python

import rospy
from dynamic_reconfigure.server import Server
from cell_culture.cfg import CellCultureGlobalConfig

def callback(config, level):
    return config

if __name__ == "__main__":
    rospy.init_node("cell_culture_global_reconfigure", anonymous=False)
    srv = Server(CellCultureGlobalConfig, callback)
    rospy.loginfo("Cell culture global reconfigure server started")
    rospy.spin()