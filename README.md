# CS-374 Algorithms Project: Shortest Path

## Overview

This project implements A* and Dijkstra’s Algorithm on OpenStreetMap data using `osmnx`.

## Driver File
The driver file shows the outputs of the algorithms visualizes them. Uses `osmns`, `networkx`, `matplotlib`, `heapq` libraries.
## `shortest_path` file
This file contains the `ShortestPath` class, which is used to build a `networkx` graph using a box. The constructor takes in starting and ending points, and buffer for the box around the points.
## Class Methods

*  `createGraph`


Helper function for building graph and adding edge speeds. Takes in `start`, `end`, `bounding_buffer` and makes a box, adding edge weights as road speeds
Returns `G`, a `networkx` graph
* 	`heuristic`


Euclidean distance heuristic used for `a_star` method. Takes in a starting`node`and `goal` and returns the Euclidean between the points
* 	`a_star`


Calculates shortest path between `start` and `end` using A*. Takes in a `traffic_weight`, which multiplies the edge weights to simulate traffic. Returns a `path` list, `total_time`, and `explored` dictionary.
* 	`dijkstra`


Calculates shortest path between `start` and `end` using Dijkstra’s algorithm Takes in a `traffic_weight`, which multiplies the edge weights to simulate traffic. Returns a `path` list, `total_time`, and `explored` dictionary.


* 	`displayRoute` 

Displays shortest path route using matplotlib. Takes in `route`, `travel_time`, `explored` and `figsize` tuple to adjust for larger routes. Optional `explored` parameter is shown in yellow. The time taken in minutes is shown near the destination node.
