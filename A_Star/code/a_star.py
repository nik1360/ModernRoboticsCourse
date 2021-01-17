import numpy as np

def init(nodes):
    past_cost = np.zeros(len(nodes))    # past cost of the nodes
    optimal_ctg = np.zeros(len(nodes))  # heuristic for the nodes
    est_tot_cost = np.zeros(len(nodes))  # total cost for the nodes
    parent_nodes = []  # list of parents for each node
    
    # initialize the arrays defined above
    for i in range(len(nodes)): 
        past_cost[i] = float('inf')
        optimal_ctg[i] = nodes[i]['h']
        est_tot_cost[i] = float('inf')
        parent_nodes.append(None)
    dic = {"past_cost": past_cost, "optimal_ctg": optimal_ctg, "est_tot_cost": est_tot_cost, "parent_nodes": parent_nodes}
    return dic


def find_connected_nodes(n_id, edges):
    # function th, for a given node, finds the connected nodes and the cost
    # of the edge
    connected = []    
    for e in edges:
        if n_id  == e["id1"]:
            connected.append((e["id2"], e["cost"]))
        if n_id  == e["id2"]:
            connected.append((e["id1"], e["cost"]))
    return connected


def choose_from_open(open_nodes):
    # choose from the open set of nodes the one with minimum total expected
    # cost
    min_node = None
    min_cost = float("inf")
    for (node, cost) in open_nodes:
        if cost < min_cost:
            min_cost = cost
            min_node = node
    return min_node

def generate_path(final_node, parents_lists):
    # analyze the parent list backward and find the path to final_node
    path = []  
    path.insert(0, final_node) 
    parent = parents_lists[final_node]
      
    while parent is not None:
        path.insert(0, parent)
        parent = parents_lists[parent]
    return path

def search_path(nodes, edges):
    alg_info = init(nodes)
    
    open_nodes = []  # nodes that must be explored
    closed_nodes = [] # nodes that has already been completely explored
    
    # Start considering node 0 and add to the open list
    curr_node = 0
    alg_info["past_cost"][curr_node] = 0
    alg_info["est_tot_cost"][curr_node] = alg_info["optimal_ctg"][curr_node]
    open_nodes.append((curr_node, alg_info["est_tot_cost"][curr_node]))
    
    # Loop until the open set of nodes is not empty
    while len(open_nodes) > 0:
        
        connected = find_connected_nodes(curr_node, edges)
        
        # update the information related to the nodes that are connected to the current one
        for (n, c) in connected:
            # ignore the nodes that belongs to the closed set
            if n not in closed_nodes:
                # update info about a node only if the path through curr_node is shorter
                if alg_info["past_cost"][curr_node] + c < alg_info["past_cost"][n]:
                    alg_info["past_cost"][n] = alg_info["past_cost"][curr_node] + c
                    alg_info["parent_nodes"][n] = curr_node 
                    alg_info["est_tot_cost"][n] = alg_info["past_cost"][n] + alg_info["optimal_ctg"][n]
                # check if the node is already present in the open set: if yes, update its cost, otherwise
                # add it to the open set                
                add = True             
                for i in range(len(open_nodes)):
                    if open_nodes[i][0] == n:
                        open_nodes[i] = (n, alg_info["est_tot_cost"][n])
                        add = False
                if add:
                    open_nodes.append((n, alg_info["est_tot_cost"][n]))
               
        # the curr_node has been completely explored, remove from open set and add it to the closed set
        open_nodes.remove((curr_node, alg_info["est_tot_cost"][curr_node]))   
        closed_nodes.append(curr_node)  

        # choose the node that must be analyzed from the open set
        curr_node = choose_from_open(open_nodes)
    
    # generate the path
    path = generate_path(len(nodes) -1, alg_info["parent_nodes"])   
    
    return(path)
        
    


    


