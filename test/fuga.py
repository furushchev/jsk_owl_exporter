#!/usr/bin/env python

from jsk_owl_exporter import *

task = "819CD86E-A4F4-11E6-AC46-C85B763717C4"

c = get_mongo_client("localhost:27017/jsk_robot_lifelog/pr1012")
agg = MongoAggregator(c, task)
n = agg.aggregate()
w = OWLWriter(n)
print w.to_string()

