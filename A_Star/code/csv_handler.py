import csv

def readNodes():
    nodes = []
    with open('../results/nodes.csv', newline='') as csvfile:
        reader = csv.reader(csvfile, delimiter=',', quotechar='|')
        for row in reader:
            n = {"id":int(row[0]) - 1, "x":float(row[1]), "y": float(row[2]), "h":float(row[3])}
            nodes.append(n)
    return nodes

def readEdges():
    edges = []
    with open('../results/edges.csv', newline='') as csvfile:
        reader = csv.reader(csvfile, delimiter=',', quotechar='|')
        for row in reader:
            e = {"id1":int(row[0]) - 1, "id2":int(row[1]) - 1, "cost": float(row[2])}
            edges.append(e)
    return edges

def writePath(path):
    for i in range(len(path)):
        path[i] += 1
    with open('../results/path.csv', 'w' ,newline='') as csvfile:
        csv.writer(csvfile).writerow(path)
