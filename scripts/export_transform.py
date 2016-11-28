#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>


import argparse
import bson
import datetime
import json
import os
import sys
from jsk_owl_exporter import *


def json_serializer(o):
    if isinstance(o, bson.ObjectId):
        return { "$oid": str(o) }
    elif isinstance(o, datetime.datetime):
        return { "$date": get_epoch_time(o) }
    raise TypeError(repr(o) + " is not JSON serializable")

def export_tf(db_addr, task_id, out_path):
    client = get_mongo_client(db_addr)
    cur = client.find({
        "_meta.stored_type": { "$in": [
            "geometry_msgs/TransformStamped",
            "posedetection_msgs/Object6DPose" ]},
        "_meta.task_id": task_id,
    }).sort("_id")

    out_dir = os.path.dirname(out_path)
    if not os.path.exists(out_dir):
        print "%s does not exists. created directories..." % out_dir
        os.makedirs(out_dir)

    print "exporting %s messages to %s" % (cur.count(), out_path)

    with open(out_path, "w") as f:
        for d in cur:
            t = d["_meta"]["stored_type"]
            if t == "posedetection_msgs/Object6DPose":
                json.dump(object6dpose_to_tf(d), f, default=json_serializer)
            elif t == "geometry_msgs/TransformStamped":
                json.dump(transform_stamped_to_tf(d), f, default=json_serializer)
            f.write(os.linesep)

    return True

def exec_command(args):
    return export_tf(args.db, args.task, args.out)


if __name__ == '__main__':
    p = argparse.ArgumentParser(description="OWL Transform Information Exporter from mongo database")
    p.add_argument("task", type=str, help="Task ID")
    p.add_argument("--out", type=str, help="output path",
                   default=os.path.join(os.getcwd(), "tf.json"))

    # mongodb
    p.add_argument("--db", default="mongodb://localhost:27017/jsk_robot_lifelog/pr1012",
                   type=str, help="address for database")

    args = p.parse_args()
    sys.exit(exec_command(args))
