import argparse
import os
from typing import List, Tuple

from Plotter import Plotter
from shapely.geometry.polygon import Polygon, LineString
import math

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
def get_visibility_graph(obstacles: List[Polygon], source=None, dest=None) -> List[LineString]:
    """
    Get The visibility graph of a given map
    :param obstacles: A list of the obstacles in the map
    :param source: The starting position of the robot. None for part 1.
    :param dest: The destination of the query. None for part 1.
    :return: A list of LineStrings holding the edges of the visibility graph
    """
    visibility_edges = []
    # nodes = [obstacle.centroid for obstacle in obstacles]
    nodes = [vertex for obstacle in obstacles for vertex in obstacle.exterior.coords[:-1]]
    for p in nodes:
        for q in nodes:
            if p == q:
                continue
            line = LineString([p, q])
            intersection_points = []
            intersection_lines = []
            for obstacle in obstacles:
                intersection = line.intersection(obstacle)
                if intersection.is_empty:
                    continue
                elif intersection.geom_type == 'Point':
                    intersection_points.append(intersection)
                elif intersection.geom_type == 'MultiPoint':
                    intersection_points.extend(intersection)
                elif intersection.geom_type == 'LineString':
                    intersection_lines.append(intersection)
                else:
                    print("Error, unhandled intersection type!")
                    exit(0xdeadbeef)
            if all(point not in intersection_points for point in [p, q]) and \
               all(line not in intersection_lines for line in [line, line.reverse()]):
                visibility_edges.append(line)
    
    return visibility_edges


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
    print("get graph")
    lines = get_visibility_graph(c_space_obstacles)
    plotter2 = Plotter()

    plotter2.add_obstacles(workspace_obstacles)
    plotter2.add_c_space_obstacles(c_space_obstacles)
    plotter2.add_visibility_graph(lines)
    plotter2.add_robot(source, dist)

    plotter2.show_graph()
    print("done showing")
    # # step 3:
    # with open(query, 'r') as f:
    #     dest = tuple(map(float, f.readline().split(',')))

    # lines = get_visibility_graph(c_space_obstacles, source, dest)
    # #TODO: fill in the next line
    # shortest_path, cost = None, None

    # plotter3 = Plotter()
    # plotter3.add_robot(source, dist)
    # plotter3.add_obstacles(workspace_obstacles)
    # plotter3.add_robot(dest, dist)
    # plotter3.add_visibility_graph(lines)
    # plotter3.add_shorterst_path(list(shortest_path))


    # plotter3.show_graph()