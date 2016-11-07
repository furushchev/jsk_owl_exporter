#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>


import argparse
import os
import pymongo
from bson.son import SON
import sys

def get_mongo_client(host, port, db, col):
    c = pymongo.MongoClient(host, port)
    return c[db][col]

def list_tasks(host, port, db, col):
    client = get_mongo_client(host, port, db, col)
    res = client.aggregate([
        { "$group" : { "_id": "$_meta.task_id" }},
    ])
    if res["ok"] == 1.0:
        print "   Task ID"
        for tid in res["result"]:
            if tid["_id"]:
                print tid["_id"]
    else:
        print "Failed to fetch task id"
        return False
    return True

def info_task(host, port, db, col, task_id):
    client = get_mongo_client(host, port, db, col)

    # task name
    task_name = client.find_one({
        "_meta.task_id": task_id,
        "_meta.task_name" : { "$exists": True }})
    if task_name is not None:
        task_name = task_name["_meta"]["task_name"]

    # date
    task_start = client.find({ "_meta.task_id": task_id }).sort([("$natural", pymongo.ASCENDING)]).limit(1)
    if task_start.alive:
        task_start = task_start.next()["_meta"]["inserted_at"].strftime("%Y/%m/%d %H:%M:%S")
    else:
        task_start = "N/A"
    task_end = client.find({ "_meta.task_id": task_id }).sort([("$natural", pymongo.DESCENDING)]).limit(1)
    if task_end.alive:
        task_end = task_end.next()["_meta"]["inserted_at"].strftime("%Y/%m/%d %H:%M:%S")
    else:
        task_end = "N/A"

    # count
    data_size = client.find({"_meta.task_id": task_id }).count()

    # stat
    stat = client.aggregate([
        { "$match" : { "_meta.task_id": task_id }},
        { "$group" : { "_id": "$_meta.stored_type",
                       "size": { "$sum" : 1 }}},
        { "$sort" : SON([("_id", pymongo.ASCENDING)])},
    ])
    if stat["ok"] != 1.0:
        stat = None
    else:
        stat = {d["_id"]: d["size"] for d in stat["result"]}

    # print result
    fmt = "{:<13} | {:<30}"
    print fmt.format("Task ID", task_id)
    print fmt.format("Task Name", task_name)
    print fmt.format("Date", task_start + " - " + task_end)
    print fmt.format("Data Size", data_size)
    print "Statistics"
    statfmt = "  {:<50} | {:>6}"
    print statfmt.format("Data Type", "Size")
    for t, s in stat.items():
        print statfmt.format(t, s)
    return True

def exec_command(args):
    if args.command == "list":
        return list_tasks(args.host, args.port, args.db, args.col)
    elif args.command == "info":
        return info_task(args.host, args.port, args.db, args.col, args.task)
    elif args.command == "export":
        pass

if __name__ == '__main__':
    p = argparse.ArgumentParser(description="OWL Exporter from mongo database")
    p.add_argument("command", type=str, help="command",
                   choices=["list", "info", "export"])

    # task
    sp = p.add_argument_group("task")
    sp.add_argument("--task", type=str, help="task id")

    # mongodb
    sp = p.add_argument_group("mongodb")
    sp.add_argument("--host", default="localhost", type=str,
                    help="hostname of mongodb server")
    sp.add_argument("--port", default=27017, type=int,
                    help="port of mongodb server")
    sp.add_argument("--db", default="test", type=str,
                    help="database name of mongodb server")
    sp.add_argument("--col", default="test", type=str,
                    help="collection name of mongodb server")

    # owl
    sp = p.add_argument_group("owl")
    sp.add_argument("--output", default=os.getcwd(), type=str,
                    help="output directory of expoerted episodic memories")


    args = p.parse_args()
    sys.exit(exec_command(args))

