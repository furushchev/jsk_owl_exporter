#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>

from bson.son import SON
import datetime
import pymongo
import random
import string


class UniqueStringGenerator(object):
    def __init__(self, strlen=8):
        self.strlen = strlen
        self.issued = set()
    def gen(self):
        while True:
            s = ''.join([random.choice(string.ascii_letters + string.digits) for i in range(self.strlen)])
            if s not in self.issued:
                self.issued |= set(s)
                return s

def get_epoch_time(dt):
    return int((dt - datetime.datetime.utcfromtimestamp(0)).total_seconds() * 1000.0)

def parse_mongodb_address(s):
    host = "localhost"
    port = 27017
    db = "test"
    col = "test"
    if s.startswith("mongodb://"):
        s = s[len("mongodb://"):]
    s = s.split("/")
    if len(s) > 3:
        return None
    elif len(s) >= 1:
        ss = s[0].split(":")
        if len(ss) >= 1:
            host = ss[0]
        elif len(ss) == 2:
            port = int(ss[1])
        else:
            return None
    if len(s) >= 2:
        db = s[1]
    if len(s) >= 3:
        col = s[2]
    return { "host": host,
             "port": port,
             "db"  : db,
             "col" : col }

def get_mongo_client(address):
    addr = parse_mongodb_address(address)
    if addr is None:
        return None
    c = pymongo.MongoClient(addr["host"], addr["port"])
    return c[addr["db"]][addr["col"]]

def list_logged_tasks(client):
    res = client.aggregate([
        { "$group": { "_id": "$_meta.task_id" }},
    ])
    tasks = []
    if res["ok"] == 1.0:
        for tid in res["result"]:
            tasks.append(tid["_id"])
    return tasks

def get_logged_task_info(client, task_id):
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

    return {
        "Task ID": task_id,
        "Task Name": task_name,
        "Date": task_start + " - " + task_end,
        "Data Size": data_size,
        "Data": stat,
    }

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

