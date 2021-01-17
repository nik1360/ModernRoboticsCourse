import a_star
import numpy as np 
import csv

# -----------------------------------------------CSV read/write functions ------------------------------------
def read_obstacles():
    obstacles = []
    with open("../results/obstacles.csv", newline='') as csv_file:
        reader = csv.reader(csv_file, delimiter=',', quotechar='|')
        for row in reader:
            o = {'x': float(row[0]), 'y':float(row[1]), 'radius':float(row[2])}
            obstacles.append(o)
    return obstacles

def write_nodes(nodes):
    with open("../results/nodes.csv", 'w', newline='' ) as csv_file:
        writer = csv.writer(csv_file)
        for n in nodes:
            writer.writerow([n['index']+1, n['x'], n['y']])
   
def write_edges(edges):
    with open("../results/edges.csv", 'w', newline='' ) as csv_file:
        writer = csv.writer(csv_file)
        for e in edges:
            writer.writerow([e['n1']+1, e['n2']+1, e['cost']])  

def write_path(path):
    for i in range(len(path)):
        path[i] += 1
    with open('../results/path.csv', 'w' ,newline='') as csvfile:
        csv.writer(csvfile).writerow(path)

# --------------------------------------------- RRT related functions ------------------------------------
def rrt(max_nodes, step_size, start_node, goal_range_x, goal_range_y, x_range, y_range, obstacles):
    nodes = [start_node] # the tree is initialized with the start node
    edges = []
    
    i_node = 1 
    goal_reached = False
    while not goal_reached:
        sample = sample_c_space(goal_range_x, goal_range_y, x_range, y_range) # sample the C-space
        nearest, distance = find_nearest_node(sample, nodes) # find the node in the tree nearest to the sample
        if distance < step_size: # if is too far, find an intermediate node
            new = sample
        else:
            new = find_new_node(sample, nearest, step_size)
        
        add_node = True
        for obs in obstacles:
            intersects = check_intersection(n1=new, n2=(nearest['x'], nearest['y']), obs=obs)
            if intersects == True:
                add_node = False            
        if add_node == True:
            # add the sampled node to the tree and define the edge
            nodes.append({'index':i_node, 'x':new[0], 'y':new[1], 'parent':nearest['index']})
            edges.append({'n1': i_node, 'n2':nearest['index'], 'cost':distance})
            i_node += 1
            # Check if the new node is in the goal region
            if (goal_range_x[0] <= new[0] <= goal_range_x[1]) and (goal_range_y[0] <= new[1] <= goal_range_y[1]):
                goal_reached = True
                print("Path to goal exists!")
        
        if len(nodes)==max_nodes and not goal_reached:
            print("Impossible to reach the goal, try increasing the number of max nodes or the step size!")
            break
            
    return nodes, edges, goal_reached

def find_new_node(sample, nearest, step_size):
    # Function that finds a point distant step_size from nearest, i nthe direction of sample
    x1 = np.array([sample[0], sample[1]])
    x2 = np.array([nearest['x'], nearest['y']])
    v_dir = (x1 - x2) / (np.linalg.norm(x1 - x2))
    
    new = x2 + step_size * v_dir
    return (new[0], new[1])
    
    

def generate_path(nodes):
    # start fro mthe last node added to the tree (the goal) and go back until the root is reached
    parent_index = float('inf')
    current_node = nodes[-1]
    path = []
    while parent_index is not None:
        path.insert(0, current_node['index'])
        parent_index = current_node['parent']
        if parent_index is not None:
            current_node = nodes[parent_index]
    return path


def find_nearest_node(sample, nodes):
    min_dist = float('inf')    
    nearest = None
    for node in nodes:
        d = np.sqrt((node['x'] - sample[0])**2 + (node['y'] - sample[1])**2)
        if d < min_dist:
            min_dist = d
            nearest = node
    return nearest, min_dist

def sample_c_space(goal_range_x, goal_range_y, x_range, y_range):
    
    epsilon = 0.05    
    if np.random.rand() > epsilon:
        # sample from the entire C-space with probability 1 - epsilon
        n = (np.random.uniform(x_range[0], x_range[1]), np.random.uniform(y_range[0], y_range[1]))
    else:
        # sample from the goal region with probability epsilon
        n = (np.random.uniform(goal_range_x[0], goal_range_x[1]), np.random.uniform(goal_range_x[0], goal_range_x[1]))
        
    return n     
       
def check_intersection(n1, n2, obs):
    # Function that checks if a segment intersects with an obstacle
    
    p1 = np.array([n1[0], n1[1]])  
    p2 = np.array([n2[0], n2[1]])       
    q = np.array([obs['x'], obs['y']])
    r = obs['radius']
    

    v = p2 - p1 # segment that goes from 
    
    # Intersections can be founded by solving the equation 
    # t^2(v⋅v) + 2t(v⋅(p1−q)) + (p1⋅p1+q⋅q−2p1⋅q−r^2) = 0
    # so it is sufficient to compute the discriminant delta and see if it is non-negative.
    
    a = np.dot(v,v)
    b = 2 * np.dot(v, p1 - q)
    c = np.dot(p1,p1) + np.dot(q,q) - 2*np.dot(p1,q) - r**2

    disc = b**2 - 4*a*c
    if disc < 0:
        return False # No intersections
        
    else:
        sqrt_disc = np.sqrt(disc)
        t1 = (-b + sqrt_disc) / (2 * a)
        t2 = (-b - sqrt_disc) / (2 * a)
        if not (0 <= t1 <= 1 or 0 <= t2 <= 1):
            return False
        else:
            return True # There is one or more inteersections

# -------------------------------- Main script ----------------------------       
    
if __name__ == "__main__":
    obstacles = read_obstacles()
    
    # range of the C-Space
    x_range = [-0.5, 0.5]
    y_range = [-0.5, 0.5]
    
    # define the starting conditions
    max_nodes = 50
    step_size = 0.1 #[m]
    start_node = {'index':0, 'x':-0.5, 'y':-0.5, 'parent':None}
    goal_pos = (0.5, 0.5)
    # define the goal area as a square that has adges long l [m] and cap if it exceedes the boundaries
    l = 0.05
    goal_range_x = [max(x_range[0], goal_pos[0] - l/2), min(x_range[1], goal_pos[0] + l/2)]
    goal_range_y = [max(y_range[0], goal_pos[1] - l/2), min(y_range[1], goal_pos[1] + l/2)]
    
    # execute rrt to generate tree    
    nodes, edges, goal_reached = rrt(max_nodes, step_size, start_node, goal_range_x, goal_range_y, x_range, y_range, obstacles)
  
    if(goal_reached):
        path = generate_path(nodes)
    else:
        path = []
    
    write_nodes(nodes)
    write_edges(edges)
    write_path(path)
    
