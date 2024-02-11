import argparse
import os
from typing import List, Tuple
import heapq
from Plotter import Plotter
from shapely.geometry.polygon import Polygon, LineString, Point
import math
from collections import defaultdict


# 0,0 1,0 1,1 0,1 
# 10,3 11,3 11,5 11,6 10,5 
# -5,-5 -3,-5 -4,0 
# 2,0 3,-1 2,-3 4,-4 3,-5 0,-4 0,-2 
# 10,0 10,1 11,2 13,1 12,-2 11,-2 

CORRECT_OBSTACLES = True

def combine_points(p1, p2):
    return (p1[0] + p2[0], p1[1] + p2[1])
    
    
def angle_with_x_axis(point1, point2):
    """
    Calculate the angle between a line segment and the positive x-axis.
    """
    dx = point2[0] - point1[0]
    dy = point2[1] - point1[1]

    # Calculate the angle using arctan2
    angle_radians = math.atan2(dy, dx)

    # Ensure the angle is positive
    if angle_radians < 0:
        angle_radians += 2 * math.pi

    return angle_radians

def array_angle(array, i):
    return angle_with_x_axis(array[i%len(array)], array[(i+1)%(len(array))])

# TODO
def get_minkowsky_sum(original_shape: Polygon, r: float) -> Polygon:
    """
    Get the polygon representing the Minkowsky sum
    :param original_shape: The original obstacle
    :param r: The radius of the rhombus
    :return: The polygon composed from the Minkowsky sums
    """
    # assumptions:
    #   we'll denote vertices of the robot - v_i. and vertices of the obstacle - w_j
    #   1) v_1 and w_1 have the smallest y-coordinates in their respective lists
    #   2) the vertices are arranged counter clock-wise.
    robot_coords = [(0,-r), (r,0),(0,r),(-r,0)]         #P
    obst_coords = original_shape.exterior.coords[:-1]   #Polygon repeats the last step
    
    mink_sum = []

    i, j = 0, 0
    while j <= len(obst_coords) and i <= len(robot_coords): 
        mink_sum.append(combine_points(obst_coords[j%len(obst_coords)], robot_coords[i%len(robot_coords)]))
        if array_angle(robot_coords,i) < array_angle(obst_coords,j): 
            i += 1
        elif array_angle(obst_coords,j) < array_angle(robot_coords,i):
            j += 1
        else:
            i += 1
            j += 1
    return Polygon(mink_sum)

# TODO
def check_visibility(line, obstacles, edges):
        count = 0
        for obst in obstacles:
            if (not line.touches(obst)) and (line.intersects(obst)):
                count += 1 # the line intersects the obstacle, and passes througt it ( not touch an outside vertex)
                break
            else:
                pass # the line either touches (an outside vertex) or doesn't intersect with the obstacle, check if it
        if count == 0:
            edges.append(line) # line doesn't intersect with obstacles
        else:
            pass # line intersects with at least one obstacle

def get_visibility_graph(obstacles: List[Polygon], source=None, dest=None) -> List[LineString]: # O(n^3)
    """
    Get The visibility graph of a given map
    :param obstacles: A list of the obstacles in the map
    :param source: The starting position of the robot. None for part 1.
    :param dest: The destination of the query. None for part 1.
    :return: A list of LineStrings holding the edges of the visibility graph
    """        
    visibility_edges = []
    vertices = [vertex for obstacle in obstacles for vertex in obstacle.exterior.coords[:-1]]
    if source is not None:
        vertices.append(source)
    if dest is not None:
        vertices.append(dest)

    for i in range(len(vertices)):
        for j in range(i + 1, len(vertices)):
            line = LineString([vertices[i], vertices[j]])
            check_visibility(line, obstacles, visibility_edges) # finds if line intersects any obstacles.
    return visibility_edges

#TODO: implement dijkstra's algorithm for shortest path, return the shortest path and its cost!
def dijkstra_with_goal(graph, start, goal):
    distances = {node: float('inf') for node in graph}
    parents = {node: None for node in graph}
    distances[start] = 0
    priority_queue = [(0, start)] # (dst, vertex)
    shortest_path, sp_cost = [], 0

    while priority_queue: # (while heap is not empty)
        current_distance, current_node = heapq.heappop(priority_queue) # pop vertex with minimal distance
        if current_node == goal:
            # it's guaranteed that there's no better path -- proof in Algorithms that when a node is processed we have the shortest path to it.
            while current_node:
                shortest_path.append(current_node)
                current_node = parents[current_node]
            shortest_path = shortest_path[::-1] # reverse the list
            break # we get a reversed list!
        if current_distance > distances[current_node]:
            continue # we have a better path to this node!
        for neighbor, weight in graph[current_node].items(): # check if we improve any of the neigbors' paths
            distance = current_distance + weight
            if distance < distances[neighbor]:
                distances[neighbor] = distance
                parents[neighbor] = current_node
                heapq.heappush(priority_queue, (distance, neighbor) )
    return shortest_path, distances[goal]

def build_graph(line_strings: List[LineString]):
    #description: the graph has a dictionary with all the vertices
    # each vertex has a dictionary housing: (Neighbor, dist)
    graph = defaultdict(dict)
    for line in line_strings:
        u, v = line.coords # each line has 2 coordinates that represent his beginning and end.
        if u not in graph:
            graph[u] = {}
        if v not in graph:
            graph[v] = {}
        graph[u][v], graph[v][u] = line.length, line.length # adds a non directed edge with its cost!
    return graph

def build_graph_and_calculate_shortest_path(visibility_lines: List[LineString], start, goal ):
    return dijkstra_with_goal(build_graph(visibility_lines), start, goal)

def is_valid_file(parser, arg):
    if not os.path.exists(arg):
        parser.error("The file %s does not exist!" % arg)


def get_points_and_dist(line):
    source, dist = line.split(' ')
    dist = float(dist)
    source = tuple(map(float, source.split(',')))
    return source, dist

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("Robot", help="A file that holds the starting position of the robot, and the distance from the center of the robot to any of its vertices")
    parser.add_argument("Obstacles", help="A file that contains the obstacles in the map")
    parser.add_argument("Query", help="A file that contains the ending position for the robot.")
    args = parser.parse_args()
    obstacles = args.Obstacles
    robot = args.Robot
    query = args.Query
    is_valid_file(parser, obstacles)
    is_valid_file(parser, robot)
    is_valid_file(parser, query)
    workspace_obstacles = []
    with open(obstacles, 'r') as f:
        for line in f.readlines():
            ob_vertices = line.split(' ')
            if ',' not in ob_vertices:
                ob_vertices = ob_vertices[:-1]
            points = [tuple(map(float, t.split(','))) for t in ob_vertices]
            workspace_obstacles.append(Polygon(points))
    with open(robot, 'r') as f:
        source, dist = get_points_and_dist(f.readline())

    # step 1:
    c_space_obstacles = [get_minkowsky_sum(p, dist) for p in workspace_obstacles]
    plotter1 = Plotter()
    plotter1.add_obstacles(workspace_obstacles)
    plotter1.add_c_space_obstacles(c_space_obstacles)
    plotter1.add_robot(source, dist)

    plotter1.show_graph()
    # step 2:
    lines = get_visibility_graph(c_space_obstacles)
    plotter2 = Plotter()

    plotter2.add_obstacles(workspace_obstacles)
    plotter2.add_c_space_obstacles(c_space_obstacles)
    plotter2.add_visibility_graph(lines)
    plotter2.add_robot(source, dist)

    plotter2.show_graph()
    # step 3:
    with open(query, 'r') as f:
        dest = tuple(map(float, f.readline().split(',')))

    lines = get_visibility_graph(c_space_obstacles, source, dest)
    #TODO: fill in the next line
    shortest_path, cost = build_graph_and_calculate_shortest_path(lines, source, dest)

    plotter3 = Plotter()
    plotter3.add_robot(source, dist)
    plotter3.add_obstacles(workspace_obstacles)
    plotter3.add_robot(dest, dist)
    plotter3.add_visibility_graph(lines)
    plotter3.add_shorterst_path(list(shortest_path))


    plotter3.show_graph()