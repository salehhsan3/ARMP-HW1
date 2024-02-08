import argparse
import os
from typing import List, Tuple

from Plotter import Plotter
from shapely.geometry.polygon import Polygon, LineString
import math

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
    robot_coords = [(0,-r), (r,0),(0,r),(-r,0)]         #P
    obst_coords = original_shape.exterior.coords[:-1]   #Polygon repeats the last step
    mink_sum = []

    i, j = 0, 0
    while i <= len(obst_coords) and j <= len(robot_coords): 
        mink_sum.append(combine_points(obst_coords[i%len(obst_coords)], robot_coords[j%len(obst_coords)]))
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
    return [(0,0),(1,1)]


def is_valid_file(parser, arg):
    if not os.path.exists(arg):
        parser.error("The file %s does not exist!" % arg)


def get_points_and_dist(line):
    source, dist = line.split(' ')
    dist = float(dist)
    source = tuple(map(float, source.split(',')))
    return source, dist

def my_main():
    parser = argparse.ArgumentParser()
    parser.add_argument("Robot", help="A file that holds the starting position of the robot, and the distance from the center of the robot to any of its vertices")
    parser.add_argument("Obstacles", help="A file that contains the obstacles in the map")
    parser.add_argument("Query", help="A file that contains the ending position for the robot.")
    args = parser.parse_args()
    obstacles = args.Obstacles
    robot = args.Robot
    query = args.Query
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
    c_space_obstacles = [get_minkowsky_sum(p, dist) for p in workspace_obstacles]
    plotter1 = Plotter()
    plotter1.add_obstacles(workspace_obstacles)
    plotter1.add_c_space_obstacles(c_space_obstacles)
    plotter1.add_robot(source, dist)

    plotter1.show_graph()

if __name__ == '__main__':
    # my_main()
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
    print("done computing")
    plotter1.add_obstacles(workspace_obstacles)
    plotter1.add_c_space_obstacles(c_space_obstacles)
    plotter1.add_robot(source, dist)

    plotter1.show_graph()
    print("done showing")
    # # step 2:

    # lines = get_visibility_graph(c_space_obstacles)
    # plotter2 = Plotter()

    # plotter2.add_obstacles(workspace_obstacles)
    # plotter2.add_c_space_obstacles(c_space_obstacles)
    # plotter2.add_visibility_graph(lines)
    # plotter2.add_robot(source, dist)

    # plotter2.show_graph()

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