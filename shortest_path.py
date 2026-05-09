import osmnx as ox
import networkx as nx
import heapq
import matplotlib.pyplot as plt


class ShortestPath:

    def __init__(self, start_coords, end_coords, bounding_buffer=0.05):

        self.start_coords = start_coords
        self.end_coords = end_coords

        # create graph helper function
        self.G = self._createGraph(start_coords, end_coords, bounding_buffer)

        # coords[1] is longitude (x), coords[0] is latitude (y)
        self.start = ox.distance.nearest_nodes(
            self.G, X=start_coords[1], Y=start_coords[0])
        self.end = ox.distance.nearest_nodes(
            self.G, X=end_coords[1], Y=end_coords[0])

    def _createGraph(self, start, end, bounding_buffer):
        north = max(start[0], end[0])+bounding_buffer
        south = min(start[0], end[0])-bounding_buffer
        east = max(start[1], end[1]) + bounding_buffer
        west = min(start[1], end[1])-bounding_buffer
        print("loading graph")
        #creating graph from box (left, bottom, right, top)
        G = ox.graph_from_bbox(
            bbox=(west, south, east, north), network_type="drive")
        G = ox.add_edge_speeds(G)
        G = ox.add_edge_travel_times(G)
        return G

    def heuristic(self, node, goal):
        # euclidean distance heuristic
        y1, x1 = self.G.nodes[node]['y'], self.G.nodes[node]['x']
        y2, x2 = self.G.nodes[goal]['y'], self.G.nodes[goal]['x']
        return ox.distance.euclidean(y1, x1, y2, x2)

    def a_star(self, traffic_weight):

        # traffic travel time, edge weight is multiplied
        for u, v, k, data in self.G.edges(data=True, keys=True):
            data['weighted_time'] = data['travel_time'] * traffic_weight
        priority_queue = [(0, self.start)]
        path = {}
        cost = {self.start: 0}
        explored = set()

        while priority_queue:
            #pop node with cheapest code
            _, current = heapq.heappop(priority_queue)
            explored.add(current)
            # if we reached the goal, add the path
            if current == self.end:
                reconstructed_path = []
                total_time = cost[self.end]
                #trace the path back
                while current in path:
                    reconstructed_path.append(current)
                    current = path[current]
                # total time in seconds
                total_time = cost[self.end]
                # return the path from start to end, time, and explored
                return reconstructed_path[::-1], total_time, explored

            # look at road neighbors
            for neighbor in self.G.neighbors(current):
                # get the travel time in seconds
                edge_data = self.G.get_edge_data(current, neighbor)
                weight = min(d["weighted_time"] for d in edge_data.values())

                new_cost = cost[current] + weight
                # if we find a new path or cheaper one, update it
                if neighbor not in cost or new_cost < cost[neighbor]:
                    cost[neighbor] = new_cost
                    priority = new_cost + self.heuristic(neighbor, self.end)
                    heapq.heappush(priority_queue, (priority, neighbor))
                    path[neighbor] = current
        print("no path found")
        return None, None, explored

    def dijkstra(self, traffic_weight):
        # traffic travel time
        for u, v, k, data in self.G.edges(data=True, keys=True):
            data['weighted_time'] = data['travel_time'] * traffic_weight
        # added priority queue
        priority_queue = [(0, self.start)]
        path = {}
        cost = {self.start: 0}
        explored=set()

        # visit the next node and update path
        while priority_queue:
            current_cost, current = heapq.heappop(priority_queue)
            explored.add(current)
            #ignore if the cost is more expensive
            if current_cost > cost.get(current, float("inf")):
                            continue
            if current == self.end:
                reconstructed_path = []
                total_time = cost[self.end]

                # add to the path
                while current in path:
                    reconstructed_path.append(current)
                    current = path[current]
                reconstructed_path.append(self.start)
                # return the path and the time it takes
                return reconstructed_path[::-1], total_time,explored

            
            for neighbor in self.G.neighbors(current):
                edge_data = self.G.get_edge_data(current, neighbor)
                weight = min(d["weighted_time"] for d in edge_data.values())
                # get the new cost
                new_cost = current_cost+weight

                # if we find a cheaper or new cost, update it
                if neighbor not in cost or new_cost < cost[neighbor]:
                    cost[neighbor] = new_cost
                    heapq.heappush(priority_queue, (new_cost, neighbor))
                    path[neighbor] = current
        # if we didn't find a path
        print("no path found")
        return None, None

    def displayRoute(self, route, travel_time, explored=None,figsize=(10,10)):
        #for cleaner plotting
        G_undirected = ox.convert.to_undirected(self.G)

        #hiding graph until the points and time are plotted
        fig, ax = ox.plot_graph_route(
            G_undirected,   route,  route_color="r", route_linewidth=4, node_size=0,
            show=False, close=False,figsize=figsize
        )
        
        # for a* explored visualization
        if explored:
            explored_x = [self.G.nodes[n]["x"] for n in explored]
            explored_y = [self.G.nodes[n]["y"] for n in explored]
            ax.scatter(explored_x, explored_y, c="orange",
                       s=1, alpha=0.4, label="explored")
        #marking start and end points
        ax.scatter(self.start_coords[1], self.start_coords[0], c="blue",
                   s=250, label="start", zorder=5)
        ax.scatter(self.end_coords[1], self.end_coords[0],
                   c="red", s=250, label="goal", zorder=5, marker="*")
        #travel time box
        ax.annotate(f"{travel_time/60:.2f} minutes", (self.end_coords[1], self.end_coords[0]),
                    ha="center", textcoords="offset points", xytext=(0, 30), bbox=dict(facecolor="white"))
        plt.legend()
        plt.show()