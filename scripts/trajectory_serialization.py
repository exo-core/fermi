#!/usr/bin/env python

from moveit_cartesian_plan_plugin.srv import *
import rospy
import rospkg
import os
import os.path
import string
from rospy_message_converter import message_converter
from rospy_message_converter import json_message_converter
from moveit_msgs.msg import RobotTrajectory

def handle_export_trajectory(req):
    file_name = resolveFileName(req.file_name)
    rospy.loginfo("Exporting trajectory to %s", file_name)

    if os.path.exists(file_name):
        # file exists

        if os.path.isdir(file_name):
            rospy.logerr("Target file is a directory.")
            return ExportTrajectoryResponse(success=False, error="Target file is a directory.")
        elif not (req.overwrite):
            rospy.logerr("Target file alreay exists.")
            return ExportTrajectoryResponse(success=False, error="Target file alreay exists.")

        rospy.logwarn("Target file alreay exists - Overwriting...")
        os.remove(file_name)

    file = open(file_name, "w")
    file.write(json_message_converter.convert_ros_message_to_json(req.trajectory))
    file.close()
    return ExportTrajectoryResponse(success=True)

def handle_import_trajectory(req):
    file_name = resolveFileName(req.file_name)

    rospy.loginfo("Loading trajectory from %s", file_name)

    if not os.path.exists(file_name):
        rospy.logerr("File not found.")
        return ImportTrajectoryResponse(success=False, error="File not found.")

    if os.path.isdir(file_name):
        rospy.logerr("Target file is a directory.")
        return ImportTrajectoryResponse(success=False, error="Target file is a directory.")


    file = open(file_name, "r")
    content = file.read()
    file.close()

    msg_type = "moveit_msgs/RobotTrajectory"
    trajectory = json_message_converter.convert_json_to_ros_message(msg_type, content)

    return ImportTrajectoryResponse(success=True, trajectory=trajectory)

def resolveFileName(file_name):
    copy = file_name.split("://", 1)

    if len(copy) > 1 and copy[0] == "package":
        copy = copy[1].split("/", 1)
        rospack = rospkg.RosPack()
        return rospack.get_path(copy[0])+"/"+copy[1]

    return file_name


def trajectory_serialization_server():
    rospy.init_node('trajectory_serialization_server')
    exportService = rospy.Service('export_trajectory', ExportTrajectory, handle_export_trajectory)
    importService = rospy.Service('import_trajectory', ImportTrajectory, handle_import_trajectory)
    print "Trajectory Serialization ready."

    rospy.spin()

if __name__ == "__main__":
    trajectory_serialization_server()
