#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>


import pymongo


class Node(object):
    def __init__(self):
        pass

class EpisodeAggregator(object):
    def __init__(self, host, port, db_name, col_name):
        self.db = pymongo.MongoClient(host, port)[db_name][col_name]
    def tasks(self, limit=10):
        return self.db.aggregate([
            { "$match": {
                "_meta.stored_type": "jsk_demo_common/FunctionEvent",
                "_meta.status": "start" }},
            { "$group": {
                "_id": "$_meta.inserted_by",
                "count": { "$sum": 1 }}},
            { "$sort": {
                "_meta.inserted_at": -1 }},
            { "$limit": limit },
        ])
    def extract_task(self):
        pass

if __name__ == '__main__':
    agg = EpisodeAggregator("133.11.216.42", 27017, "jsk_robot_lifelog", "pr1012")
    print agg.tasks()
