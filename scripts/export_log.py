#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>


import argparse
import os
import sys
from jsk_owl_exporter import *


def list_tasks(db_addr):
    client = get_mongo_client(db_addr)
    tasks = list_logged_tasks(client)
    if len(tasks) > 0:
        print "   Task ID"
        for t in tasks:
            print t
        return True
    else: return False

def info_task(db_addr, task_id):
    client = get_mongo_client(db_addr)
    info = get_logged_task_info(client, task_id)
    stat = info.pop("Data")

    # print result
    fmt = "{:<13} | {:<30}"
    for t, s in info.items():
        print fmt.format(t, s)
    print "Statistics"
    statfmt = "  {:<50} | {:>6}"
    print statfmt.format("Data Type", "Size")
    for t, s in stat.items():
        print statfmt.format(t, s)
    return True

def export_task(db_addr, task_id, output_dir):
    client = get_mongo_client(db_addr)
    agg = MongoAggregator(client, task_id)
    node = agg.aggregate()
    writer = OWLWriter(node)
    save_path = os.path.join(output_dir, "log.owl")
    writer.to_file(save_path)
    print "saved to %s" % save_path
    return True

def exec_command(args):
    if args.command == "list":
        return list_tasks(args.db)
    elif args.command == "info":
        return info_task(args.db, args.task)
    elif args.command == "export":
        return export_task(args.db, args.task, args.output)

if __name__ == '__main__':
    p = argparse.ArgumentParser(description="OWL Exporter from mongo database")
    p.add_argument("command", type=str, help="command",
                   choices=["list", "info", "export"])

    # task
    sp = p.add_argument_group("task")
    sp.add_argument("--task", type=str, help="task id")

    # owl
    sp = p.add_argument_group("owl")
    sp.add_argument("--output", default=os.getcwd(), type=str,
                    help="output directory of expoerted episodic memories")
    # mongodb
    p.add_argument("--db", default="mongodb://localhost:27017/jsk_robot_lifelog/pr1012",
                   type=str, help="address for database")

    args = p.parse_args()
    sys.exit(exec_command(args))

