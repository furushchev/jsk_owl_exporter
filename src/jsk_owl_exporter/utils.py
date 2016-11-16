#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>

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

def get_mongo_client(host, port, db, col):
    c = pymongo.MongoClient(host, port)
    return c[db][col]


def get_epoch_time(dt):
    return int((dt - datetime.datetime.utcfromtimestamp(0)).total_seconds() * 1000.0)
