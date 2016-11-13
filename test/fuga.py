#!/usr/bin/env python

from jsk_owl_exporter import *

host = "localhost"
port = 27017
db = "jsk_robot_lifelog"
col = "pr1012"
task = "819CD86E-A4F4-11E6-AC46-C85B763717C4"

c = get_mongo_client(host, port, db, col)
agg = MongoAggregator(c, task)
n = agg.aggregate()
w = OWLWriter(n)
print w.to_string()
