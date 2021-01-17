import csv_handler as csvh
import a_star 

nodes = csvh.readNodes()
edges = csvh.readEdges()
path = a_star.search_path(nodes, edges)

csvh.writePath(path)
