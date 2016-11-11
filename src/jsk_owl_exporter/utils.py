#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>


import pymongo


def get_mongo_client(host, port, db, col):
    c = pymongo.MongoClient(host, port)
    return c[db][col]
