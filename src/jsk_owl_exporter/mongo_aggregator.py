#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>


import pymongo


class Node(object):
    def __init__(self, name, prop={}):
        self.next = None
        self.children = []
        self.name = name
        self.properties = prop
    def add_next(self, n):
        self.next = n
    def add_subnode(self, n):
        self.children.append(n)
    def add_prop(self, k, v):
        self.properties.update({k:v})


class MongoAggregator(object):
    def __init__(self, mongo_client, task):
        self.client = mongo_client
        self.task = task
        self.root_node = None
    def aggregate(self):
        res = self.client.find({"_meta.task_id": self.task }).sort([("$natural", pymongo.ASCENDING)])
        for data in res:
            if data["_meta.stored_type"] == "jsk_robot_startup/ActionEvent":
                aggregate_action()
