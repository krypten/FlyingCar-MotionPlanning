import numpy as np 
import networkx as nx

from bresenham import bresenham
from sklearn.neighbors import KDTree


class RRT:
    def __init__(self, x_init):
        # A tree is a special case of a graph with
        # directed edges and only one path to any vertex.
        self.tree = nx.DiGraph()
        self.tree.add_node(x_init)
        self.start = x_init
                
    def add_vertex(self, x_new):
        self.tree.add_node(tuple(x_new))
    
    def add_edge(self, x_near, x_new, u):
        self.tree.add_edge(tuple(x_near), tuple(x_new), orientation=u)
    
    def remove_edge(self, x_near, x_new):
        self.tree.remove_edge(tuple(x_near), tuple(x_new))
    
    def parent(self, x):
        return list(self.tree.predecessors(tuple(x)))[0]
        
    def cost(self, source_node, node):
        return np.linalg.norm(np.array(source_node) - np.array(node))
    
    def path_cost(self, path):
        total_cost = 0
        for i in range(1, len(path)):
            total_cost += self.cost(path[i-1], path[i])
        return total_cost
    
    def shortest_path(self, node):
        return nx.shortest_path(self.tree, source=self.start, target=tuple(node))
    
    @property
    def vertices(self):
        return self.tree.nodes()
    
    @property
    def edges(self):
        return self.tree.edges()

def sample_state(grid):
    x = np.random.uniform(0, grid.shape[0])
    y = np.random.uniform(0, grid.shape[1])
    return (x, y)

def sample_free_state(grid):
    x_rand = sample_state(grid)
    # sample states until a free state is found
    while grid[int(x_rand[0]), int(x_rand[1])] == 1:
        x_rand = sample_state(grid)
    return x_rand

def select_input(x_rand, x_near):
    return np.arctan2(x_rand[1] - x_near[1], x_rand[0] - x_near[0])

def new_state(x_near, u, dt):
    nx = x_near[0] + np.cos(u) * dt
    ny = x_near[1] + np.sin(u) * dt
    return [nx, ny]

def nearest_neighbor(x_rand, rrt):
    closest_dist = 100000
    closest_vertex = None
    x_rand = np.array(x_rand)
    
    for v in rrt.vertices:
        d = np.linalg.norm(x_rand - np.array(v[:2]))
        if d < closest_dist:
            closest_dist = d
            closest_vertex = v
    return closest_vertex

def near_neighbors(x, rrt, alpha=2):
    neighbors = []
    x_ver = np.array(x)
    
    for v in rrt.vertices:
        d = np.linalg.norm(x_ver - np.array(v[:2]))
        if d < alpha:
            neighbors.append(v)
    return neighbors

def obstacle_free(grid, existing_node, new_node):
    line = [int(existing_node[0]), int(existing_node[1]), int(new_node[0]), int(new_node[1])]
    cells = list(bresenham(line[0], line[1], line[2], line[3]))
    for cell in cells:
        if grid[int(cell[0]), int(cell[1])] == 1:
            return False
    return True

def choose_best_parent(grid, x_nearest, x_new, X_near, rrt):
    x_min = x_nearest
    best_cost = 1000000#rrt.short_length(x_min) + rrt.short_length(x_min, x_new)
    for x_near in X_near:
        if obstacle_free(grid, x_near, x_new):
            cost = rrt.path_cost(rrt.shortest_path(x_near)) + rrt.cost(x_near, x_new)
            if cost < best_cost:
                best_cost = cost
                x_min = x_near
    return x_min

def rewire_vertices(grid, x_new, X_near, rrt):
    for v in X_near:
        if obstacle_free(grid, v, x_new) and \
        rrt.path_cost(rrt.shortest_path(v)) > (rrt.path_cost(rrt.shortest_path(x_new)) + rrt.cost(v, x_new)):
            print(v)
            x_parent = rrt.parent(v)
            rrt.remove_edge(x_parent, v)
            u_parent = select_input(v, x_new)
            rrt.add_edge(x_new, v, u_parent)

def generate_RRT_star(grid, x_init, num_vertices, dt):
    
    rrt = RRT(x_init)
    
    for _ in range(num_vertices):
        
        x_rand = sample_free_state(grid)
        x_nearest = nearest_neighbor(x_rand, rrt)
        u = select_input(x_rand, x_nearest)
        x_new = new_state(x_nearest, u, dt)
        
        if obstacle_free(grid, x_nearest, x_new):
            X_near = near_neighbors(x_new, rrt)
            # Assign minimal cost neighbor
            x_min = choose_best_parent(grid, x_nearest, x_new, X_near, rrt)
            rrt.add_edge(x_min, x_new, u)
            
            # Try x_new as parent for near neigbors
            rewire_vertices(grid, x_new, X_near, rrt)
        
    return rrt

def connect(grid, x_new, x_conn, bi_rrt, dt):
    u = select_input(x_conn, x_new)
    z_new = new_state(x_new, u, dt)
    if obstacle_free(grid, x_conn, z_new) and obstacle_free(grid, z_new, x_new):
        Z_near = near_neighbors(z_new, bi_rrt[1])
        z_min = choose_best_parent(grid, x_conn, z_new, Z_near, bi_rrt[1])
        bi_rrt[1].add_edge(z_min, z_new, u)
        return bi_rrt[0].shortest_path(x_new) + bi_rrt[1].shortest_path(z_min)
    return None

def potential_force(x_goal, x_prand, potential_gain):
    return -1. * potential_gain * np.abs(x_prand - x_goal)

def bi_potential_gradient(grid, x_rand, x_goal):
    x_prand = x_rand
    return x_prand

def generate_PB_RRT(grid, x_init, x_goal, num_vertices, dt):
    
    path_best = None
    bi_rrt = [RRT(x_init), RRT(x_goal)]
    
    for _ in range(num_vertices):
        x_rand = sample_free_state(grid)
        x_prand = bi_potential_gradient(grid, x_rand, bi_rrt[1].start)
        x_nearest = nearest_neighbor(x_prand, bi_rrt[0])
        u = select_input(x_prand, x_nearest)
        x_new = new_state(x_nearest, u, dt)
        
        if obstacle_free(grid, x_nearest, x_new):
            X_near = near_neighbors(x_new, bi_rrt[0])
            # Assign minimal cost neighbor
            x_min = choose_best_parent(grid, x_nearest, x_new, X_near, bi_rrt[0])
            bi_rrt[0].add_edge(x_min, x_new, u)
            
            # Try x_new as parent for near neigbors
            rewire_vertices(grid, x_new, X_near, bi_rrt[0])
        
            x_conn = nearest_neighbor(x_new, bi_rrt[1])
            path_new = connect(grid, x_new, x_conn, bi_rrt, dt)
            if path_new is not None:
                if path_best is None:
                    path_best = path_new
                elif bi_rrt[1].path_cost(path_new) < bi_rrt[1].path_cost(path_best):
                    path_best = path_new
        
        # Swap trees
        bi_rrt[0], bi_rrt[1] = bi_rrt[1], bi_rrt[0]
    
    return bi_rrt, path_best

def path_planing_bi_rrt(grid, x_init, x_goal):
    # Parameters
    num_vertices = 1500
    dt = 1
    # Generate BiRRT*
    optimal_path = None
    count = 0
    #while optimal_path == None and count < 4:
    _, optimal_path = generate_PB_RRT(grid, x_init, x_goal, num_vertices, dt)
        # Increase the number of verticles
    #    num_vertices *= 4
    #    count += 1
    
    return optimal_path
