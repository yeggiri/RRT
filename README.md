# RRT
장애물을 피해가며 목표도달
import random as r
import math as m
import matplotlib.pyplot as mp

class Node:
    def __init__(self, x, y):
        self.x=x
        self.y=y
        self.parent = None

class Obstacle:
    def __init__(self,x,y,radius):
        self.x=x
        self.y=y
        self.radius = radius
        self.is_Obstacle = True

def check_collision(node, obstacle):
    distance = ed(node, obstacle)
    return distance < obstacle.radius



def ed(node1, node2):
    return m.sqrt((node1.x-node2.x)**2 + (node1.y - node2.y)**2)

def grn(x_max, y_max):
    x = r.uniform(0, x_max)
    y = r.uniform(0, y_max)
    return Node(x,y)

def gnn(rrt_nodes, rand_node):
    min_distance = float('inf')
    nearest_node = None
    for node in rrt_nodes:
        distance = ed(node, rand_node)
        if distance < min_distance:
            min_distance = distance
            nearest_node = node
    return nearest_node
    
def create_nd(nearest_node, rand_node, step_size, obstacles):
    theta = m.atan2(rand_node.y - nearest_node.y, rand_node.x - nearest_node.x)
    new_x = nearest_node.x + step_size * m.cos(theta)
    new_y = nearest_node.y + step_size * m.sin(theta)
    new_node = Node(new_x, new_y)
    new_node.parent = nearest_node
    
    # check for collision with obstacles
    for obstacle in obstacles:
        if check_collision(new_node, obstacle):
            return None
    
    return new_node
def goal(goal_node, current_node, goal_reaius):
    return ed(goal_node, current_node) < goal_reaius

def rrt(start_node, goal_node, x_max, y_max, step_size, goal_reaius, max_iterations, obstacles):
    rrt_nodes = [start_node]
    for _ in range(max_iterations):
        rand_node = grn(x_max, y_max)
        nearest_node = gnn(rrt_nodes, rand_node)
        new_node = create_nd(nearest_node, rand_node, step_size, obstacles)
        if new_node:
            rrt_nodes.append(new_node)
            
            if goal(goal_node, new_node, goal_reaius):
                print("GOAL")
                return rrt_nodes, new_node
    
    print("Max iterations reached, goal not found")
    return rrt_nodes, None
    


def plot_rrt(rrt_nodes, start_node, goal_node, path_node=None):  
    mp.figure()
    for node in rrt_nodes:
        if node.parent:
            mp.plot([node.x, node.parent.x], [node.y, node.parent.y], '-k')

    mp.plot(start_node.x, start_node.y, 'bo', markersize=10, label='start')
    mp.plot(goal_node.x, goal_node.y, 'go', markersize=15, label='Goal')

    if path_node:
        while path_node.parent:
            mp.plot([path_node.x, path_node.parent.x], [path_node.y, path_node.parent.y], '-r', linewidth=2)
            path_node = path_node.parent
    
    for obstacle in obstacles:
        circle = mp.Circle((obstacle.x, obstacle.y), obstacle.radius, color='red')
        mp.gca().add_artist(circle)


    mp.legend()
    mp.axis([0, x_max,0, y_max])
    mp.grid(True)
    mp.show()

if __name__ == "__main__":
    start_node = Node(1,1)
    goal_node = Node(9,9)
    x_max, y_max = 10,10
    step_size = 0.5
    goal_reaius = 0.5
    max_iterations = 1000
    
    obstacles = [
        Obstacle(4, 4, 1),
        Obstacle(7, 6, 1),
        Obstacle(5, 8, 1),
    ]

    rrt_nodes, path_node = rrt(start_node, goal_node, x_max, y_max, step_size, goal_reaius, max_iterations, obstacles)
    plot_rrt(rrt_nodes, start_node, goal_node, path_node)
