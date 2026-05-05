import osmnx as ox
import networkx as nx
import heapq
import matplotlib.pyplot as plt


class ShortestPath:

    def __init__(self, start_coords, end_coords, bounding_buffer=0.05):

        self.G = self._createGraph(start_coords, end_coords, bounding_buffer)
        

        self.start = ox.distance.nearest_nodes(self.G, X=start_coords[1], Y=start_coords[0])
        self.end = ox.distance.nearest_nodes(self.G, X=end_coords[1], Y=end_coords[0])


    def _createGraph(self, start, end, bounding_buffer):
        north = max(start[0], end[0])+bounding_buffer
        south = min(start[0], end[0])-bounding_buffer
        east = max(start[1], end[1]) + bounding_buffer
        west = min(start[1], end[1])-bounding_buffer
        print("loading graph")
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

        # traffic travel time
        for u, v, k, data in self.G.edges(data=True, keys=True):
            data['travel_time'] = data['travel_time'] * traffic_weight
        priority_queue = [(0, self.start)]
        path = {}
        cost = {self.start: 0}
        explored = set()

        while priority_queue:
            _, current = heapq.heappop(priority_queue)
            explored.add(current)
            # if we reached the goal, add the path
            if current == self.end:
                reconstructed_path = []
                while current in path:
                    reconstructed_path.append(current)
                    current = path[current]
                reconstructed_path.append(self.start)
                # total time in seconds
                total_time = cost[self.end]
                return reconstructed_path, total_time, explored

            # look at road neighbors
            for neighbor in self.G.neighbors(current):
                # get the travel time in seconds
                edge_data = self.G.get_edge_data(current, neighbor)
                weight = min(d["travel_time"] for d in edge_data.values())

                new_cost = cost[current] + weight
                # if we find a new path or cheaper one, update it
                if neighbor not in cost or new_cost < cost[neighbor]:
                    cost[neighbor] = new_cost
                    priority = new_cost + self.heuristic(neighbor, self.end)
                    heapq.heappush(priority_queue, (priority, neighbor))
                    path[neighbor] = current
        return None, None, explored

    def dijkstra(self,traffic_weight):
        # traffic travel time
        for u, v, k, data in self.G.edges(data=True, keys=True):
            data['travel_time'] = data['travel_time'] * traffic_weight
        # setting up shortest path, visited set
        shortest_paths = {self.start: (None, 0)}
        current_node = self.start
        visited = set()
        # visit the next node and update path
        while current_node != self.end:
            visited.add(current_node)
            destinations = self.G[current_node]
            weight_to_current_node = shortest_paths[current_node][1]

            for next_node in destinations:
                # get weight of the node, 0 is the first edge
                weight = self.G[current_node][next_node][0]['travel_time']
                new_weight = weight_to_current_node + weight
                # update the shortest path
                if next_node not in shortest_paths or new_weight < shortest_paths[next_node][1]:
                    shortest_paths[next_node] = (current_node, new_weight)

            next_destinations = {
                node: shortest_paths[node] for node in shortest_paths if node not in visited}

            if not next_destinations:
                return "Route Not Possible"
            # get the shortest path for the node
            current_node = min(next_destinations,
                               key=lambda k: next_destinations[k][1])

        # after the loop, if end in shortest_paths, we found a path
        if self.end in shortest_paths:
            return shortest_paths[self.end][1]
        else:
            return "Route Not Possible"

    def displayRoute(self, start, travel_time, route, explored=None):

        G_undirected = ox.convert.to_undirected(self.G)
        explored_x = [self.G.nodes[n]["x"] for n in explored]
        explored_y = [self.G.nodes[n]["y"] for n in explored]

        fig, ax = ox.plot_graph_route(
            G_undirected, route, show=False, close=False)

        ax.scatter(explored_x, explored_y, c="orange",
                   s=1, alpha=0.4, label="explored")
        ax.scatter(start[1], start[0], c="blue",
                   s=250, label="start", zorder=5)
        ax.scatter(self.end[1], self.end[0],
                   c="red", s=250, label="goal", zorder=5, marker="*")

        ax.annotate(f"{travel_time/60:.2f} minutes", (self.end[1], self.end[0]),
                    ha="center", textcoords="offset points", xytext=(0, 30), bbox=dict(facecolor="white"))
        plt.legend()
        plt.show()
