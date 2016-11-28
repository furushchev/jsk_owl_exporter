from jsk_owl_exporter import *

task = "819CD86E-A4F4-11E6-AC46-C85B763717C4"

c = get_mongo_client("mongodb://localhost:27017/jsk_robot_lifelog/pr1012")
agg = MongoAggregator(c, task)
n = agg.aggregate()
d = GraphWriter(n)
d.parse()
d.save_pdf("/tmp/graph")
