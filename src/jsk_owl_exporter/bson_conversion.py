#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>


import datetime
import rospy
from robot_state_publisher.srv import (GetRobotTransforms, GetRobotTransformsRequest)


def datetime_to_rostime(dt):
    total = (dt - datetime.datetime.utcfromtimestamp(0)).total_seconds()
    secs = int(total)
    nsecs = int((total - secs) * 1000000000)
    return { u"secs": secs, u"nsecs": nsecs }


class BSONConversion(object):
    def __init__(self, srv_name="get_transforms"):
        self.get_transforms_srv = rospy.ServiceProxy(srv_name, GetRobotTransforms)
    def to_tf_json(self, doc):
        t = doc["_meta"]["stored_type"]
        if   t == "posedetection_msgs/Object6DPose":
            return self.object6dpose_to_tf(doc)
        elif t == "geometry_msgs/TransformStamped":
            return self.transform_stamped_to_tf(doc)
        elif t == "move_base_msgs/MoveBaseActionFeedback":
            return self.move_base_action_feedback_to_tf(doc)
        elif t == "control_msgs/FollowJointTrajectoryActionFeedback":
            return self.follow_joint_trajectory_feedback_to_tf(doc)
        elif t == "sensor_msgs/JointState":
            return self.joint_state_to_tf(doc)

    def transform_stamped_array_to_tf(self, ts_arr):
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

    def transform_stamped_to_tf(self, ts):
        return self.transform_stamped_array_to_tf([ts])

    def object6dpose_to_tf(self, od, robot_frame_id="base_footprint"):
        return {
            "_meta": od["_meta"],
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
            }]}

    def move_base_action_feedback_to_tf(self, f, robot_frame_id="base_footprint"):
        base = f["feedback"]["base_position"]
        return {
            "_meta": f["_meta"],
            "transforms": [{
                "header": base["header"],
                "child_frame_id": robot_frame_id,
                "transform": {
                    "translation": base["pose"]["position"],
                    "rotation": base["pose"]["orientation"],
                },
            }]}

    def joint_state_to_tf(self, js):
        req = GetRobotTransformsRequest()
        req.joint_states.header.stamp.secs = js["header"]["stamp"]["secs"]
        req.joint_states.header.stamp.nsecs = js["header"]["stamp"]["nsecs"]
        req.joint_states.header.seq = js["header"]["seq"]
        req.joint_states.name = js["name"]
        req.joint_states.position = js["position"]
        req.joint_states.velocity = js["velocity"]
        req.joint_states.effort = js["effort"]
        res = self.get_transforms_srv(req).transforms
        return {
            "_meta": js["_meta"],
            "transforms": [{
                "header": {
                    "seq": t.header.seq,
                    "frame_id": t.header.frame_id,
                    "stamp": {
                        "secs": t.header.stamp.secs,
                        "nsecs": t.header.stamp.nsecs,
                    }},
                "child_frame_id": t.child_frame_id,
                "transform": {
                    "translation": {
                        "x": t.transform.translation.x,
                        "y": t.transform.translation.y,
                        "z": t.transform.translation.z },
                    "rotation": {
                        "x": t.transform.rotation.x,
                        "y": t.transform.rotation.y,
                        "z": t.transform.rotation.z,
                        "w": t.transform.rotation.w }}} for t in res]}


    def follow_joint_trajectory_feedback_to_tf(self, f):
        fb = f["feedback"]
        req = GetRobotTransformsRequest()
        req.joint_states.header.stamp.secs = fb["header"]["stamp"]["secs"]
        req.joint_states.header.stamp.nsecs = fb["header"]["stamp"]["nsecs"]
        req.joint_states.header.seq = fb["header"]["seq"]
        req.joint_states.name = fb["joint_names"]
        req.joint_states.position = fb["actual"]["positions"]
        req.joint_states.velocity = fb["actual"]["velocities"]
        res = self.get_transforms_srv(req).transforms
        return {
            "_meta": f["_meta"],
            "transforms": [{
                "header": {
                    "seq": t.header.seq,
                    "frame_id": t.header.frame_id,
                    "stamp": {
                        "secs": t.header.stamp.secs,
                        "nsecs": t.header.stamp.nsecs,
                    }},
                "child_frame_id": t.child_frame_id,
                "transform": {
                    "translation": {
                        "x": t.transform.translation.x,
                        "y": t.transform.translation.y,
                        "z": t.transform.translation.z },
                    "rotation": {
                        "x": t.transform.rotation.x,
                        "y": t.transform.rotation.y,
                        "z": t.transform.rotation.z,
                        "w": t.transform.rotation.w }}} for t in res]}
