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
            weight (int): The weight of the edge.
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

    def add_edge(self, weight, target_vertex):
        """
        Adds a new edge from the current vertex to the target vertex.

        Args:
            weight (int): The weight of the edge.
            target_vertex (Vertex): The target vertex of the edge.
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
    
        print(
              f"Calculating the shortest path from {source_vertex} to {end_vertex}"
        )

        self.calculate(source_vertex)

        stack = []
        while end_vertex is not None:
            stack.append(end_vertex)
            end_vertex = end_vertex.predecessor
        
        shortest_path = ""            
        while stack:
            vertex = stack.pop()
            shortest_path += f"{vertex} -> "
        
        # Remove the last three elements, i.e., "-> ".
        shortest_path = shortest_path[:-3]

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

            # If the current vertex has already been visited, we don't need
            # to process it again.
            if current_vertex.visited:
                continue

            # Traverse the neighbors of the current vertex.
            for edge in current_vertex.neighbours:
                # Get the source and target vertices for the neighbor edge.
                source_vertex = edge.source_vertex
                target_vertex = edge.target_vertex
                
                # Calculate the new distance.
                new_distance = current_vertex.min_distance + edge.weight

                # Check if the newly calculated distance is less than
                # the actual min distance for the target vertex.
                if new_distance < target_vertex.min_distance:
                    # Update the target vertex's min_distance to be
                    # the new distance.
                    target_vertex.min_distance = new_distance
                    
                    # Update the predecessor for the target vertex to be
                    # the current vertex since we found a shorter distance
                    # which might be the shortest path.
                    target_vertex.predecessor = source_vertex
                    
                    # Add the new target vertex min distance to the heap
                    # invariant. Note we don't update the min distance value
                    # for the target vertex in place in the heap since
                    # the worst would be O(n + log(n)). Instead, we add the
                    # smaller min distance value to the heap while maintaining
                    # the heap invariant, e.g., [F-13, F-15].
                    heapq.heappush(self.heap, target_vertex)

            # Mark the current vertex as visited to avoid revisiting it
            # since we always get the updated min distance for this vertex first.
            current_vertex.visited = True
