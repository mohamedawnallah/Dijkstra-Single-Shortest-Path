"""
Module implementing Dijkstra's algorithm for finding the shortest path
in a graph.
"""

import heapq

class Edge:
    def __init__(self, source_vertex, target_vertex, weight):
        """
        Represents an edge in the graph.

        Args:
            source_vertex (Vertex): The source vertex of the edge.
            target_vertex (Vertex): The target vertex of the edge.
            weight (int): The non-negative weight of the edge.
        """
        self.source_vertex = source_vertex
        self.target_vertex = target_vertex
        self.weight = weight

class Vertex:
    def __init__(self, label):
        """
        Represents a vertex in the graph.

        Args:
            label (str): The label of the vertex.
        """
        self.label = label
        self.visited = False
        self.predecessor = None
        self.neighbours = []
        self.min_distance = float('inf')

    def __repr__(self):
        return self.label

    def __lt__(self, other_vertex):
        """
        Compares vertices based on their minimum distance for heap operations.
        """
        return self.min_distance < other_vertex.min_distance

    def add_edge(self, target_vertex, weight):
        """
        Adds a new edge from the current vertex to the target vertex.

        Args:
            target_vertex (Vertex): The target vertex of the edge.
            weight (int): The weight of the edge.
        """
        new_edge = Edge(self, target_vertex, weight)
        self.neighbours.append(new_edge)


class DijkstraAlgorithm:
    def __init__(self):
        self.heap = []

    def calculate_shortest_path(self, source_vertex, target_vertex):
        """
        Calculates the shortest path from the source vertex to the target vertex.

        Args:
            source_vertex (Vertex): The source vertex.
            target_vertex (Vertex): The target vertex.

        Returns:
            str: A string representing the shortest path and its minimum distance.
        """
        end_vertex = target_vertex
    
        print(f"Calculating the shortest path from {source_vertex} to {end_vertex}")

        # Calculate the optimal tentative distances.
        self.calculate(source_vertex)

        # Get the shortest path backward from the destination.
        shortest_path = []
        while end_vertex is not None:
            shortest_path.append(end_vertex)
            end_vertex = end_vertex.predecessor
        
        return shortest_path, target_vertex.min_distance

    def calculate(self, start_vertex):
        """
        Calculates the shortest path using Dijkstra's algorithm.

        Args:
            start_vertex (Vertex): The starting vertex for the algorithm.
        """
        start_vertex.min_distance = 0
        heapq.heappush(self.heap, start_vertex)
        
        while self.heap:
            # Pop the vertex with min distance from the heap.
            current_vertex = heapq.heappop(self.heap)

            # If the current vertex has already been visited, it is because
            # nodes can have multiple edges. Hence, we do not need to recompute
            # the shortest path to this already visited node.
            if current_vertex.visited:
                continue

            # Update the tentative distances for current vertex neighbours.
            for edge in current_vertex.neighbours:
                source_vertex, target_vertex  = edge.source_vertex, edge.target_vertex
                
                # Caclulate the new distance and Check if it is less than the
                # target neigbour vertex cost.
                new_distance = current_vertex.min_distance + edge.weight
                if new_distance < target_vertex.min_distance:
                    target_vertex.min_distance = new_distance
                    target_vertex.predecessor = source_vertex
                    heapq.heappush(self.heap, target_vertex)

            current_vertex.visited = True

    @staticmethod
    def print_path(path):
        shortest_path = ""
        min_distance = path[0].min_distance

        while path:
            vertex = path.pop()
            shortest_path += f"{vertex} -> "
        shortest_path = shortest_path[:-3]

        print(f"Shortest path: {shortest_path}" + "\n" + f"Min Distance: {min_distance}")
