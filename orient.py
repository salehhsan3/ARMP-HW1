from HW1 import array_angle
import argparse
import os
from typing import List, Tuple

from Plotter import Plotter
from shapely.geometry.polygon import Polygon, LineString
import math


def rotate(L, n):
    if len(L) != 0:
        shift = n % len(L)
        for i in range(shift):
            L.append(L.pop(0))

def correct_polygon(original_shape: Polygon):
    
    obst_coords = original_shape.exterior.coords[:-1]   #Polygon repeats the last step
    
    if obst_coords[0][1] > min(obst_coords,key=lambda point: point[1])[1]:
        print("first point is not min, rotating by",obst_coords.index(min(obst_coords,key=lambda point: point[1])))
        rotate(obst_coords, obst_coords.index(min(obst_coords,key=lambda point: point[1])))
        print(obst_coords)
    #doesn't catch all cases!
    if len(obst_coords) > 2 and array_angle(obst_coords, 0) > array_angle(obst_coords, 1):
        print("flipping order!")
        obst_coords = obst_coords[::-1]
        print(obst_coords)
    
    return original_shape

def get_obstacles():
    workspace_obstacles = []
    with open('obstacles', 'r') as f:
        for line in f.readlines():
            ob_vertices = line.split(' ')
            if ',' not in ob_vertices:
                ob_vertices = ob_vertices[:-1]
            points = [tuple(map(float, t.split(','))) for t in ob_vertices]
            workspace_obstacles.append(Polygon(points))
    
    return workspace_obstacles

# This reads from "obstacles" and writes to "corrected-obstacles" the same obstacles but fixing counter clockwise-ness and lowest-point-first
if __name__ == "__main__":
    workspace_obstacles = get_obstacles()
    corrected_obstacles = [correct_polygon(obstacle) for obstacle in workspace_obstacles]

    with open('corrected-obstacles', "w") as f:
        for obst in corrected_obstacles:
            f.write(' '.join(['{0:.2g}'.format(p[0])+","+'{0:.2g}'.format(p[1]) for p in obst.exterior.coords[:-1]]) + " \n")

