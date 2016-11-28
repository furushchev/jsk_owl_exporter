#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>


import datetime

#######################################
# rostime
#######################################

def datetime_to_rostime(dt):
    total = (dt - datetime.datetime.utcfromtimestamp(0)).total_seconds()
    secs = int(total)
    nsecs = int((total - secs) * 1000000000)
    return { u"secs": secs, u"nsecs": nsecs }

#######################################
# geometry_msgs/TransformStamped
#######################################

def transform_stamped_array_to_tf(ts_arr):
    meta = ts_arr[0].pop("_meta")
    for ts in ts_arr:
        if "_meta" in ts:
            del ts["_meta"]
        if "_id" in ts:
            del ts["_id"]
    return {
        "_meta": meta,
        "transforms": ts_arr,
    }

def transform_stamped_to_tf(ts):
    return transform_stamped_array_to_tf([ts])

#######################################
# posedetection_msgs/Object6DPose
#######################################

def object6dpose_to_tf(od, robot_frame_id="base_footprint"):
    return {
        "transforms": [{
            "header": {
                "frame_id": robot_frame_id,
                "stamp": datetime_to_rostime(od["_meta"]["inserted_at"]),
                "seq": 0,
            },
            "child_frame_id": od["type"],
            "transform": {
                "translation": od["pose"]["position"],
                "rotation": od["pose"]["orientation"],
            },
            "_meta": od["_meta"],
        }]}

#######################################
# sensor_msgs/JointState
#######################################

def joint_state_to_tf(js):
    pass

#######################################
# move_base_msgs/MoveBaseAction
#######################################

# TODO

#######################################
# control_msgs/FollowJointTrajectoryAction
#######################################

# TODO
