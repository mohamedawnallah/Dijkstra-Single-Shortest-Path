
from dijkstra import Vertex, Edge, DijkstraAlgorithm

# Create Vertices.
a_vertex = Vertex("A")
b_vertex = Vertex("B")
c_vertex = Vertex("C")
d_vertex = Vertex("D")
e_vertex = Vertex("E")
f_vertex = Vertex("F")

# Create Edges (assuming Undirected Graph).
a_b_edge = Edge(a_vertex, b_vertex, 4)
a_c_edge = Edge(a_vertex, c_vertex, 5)
b_c_edge = Edge(b_vertex, c_vertex, 11)
b_d_edge = Edge(b_vertex, d_vertex, 9)
b_e_edge = Edge(b_vertex, e_vertex, 7)
c_e_edge = Edge(c_vertex, e_vertex, 3)
d_e_edge = Edge(d_vertex, e_vertex, 13)
d_f_edge = Edge(d_vertex, f_vertex, 2)
e_f_edge = Edge(e_vertex, f_vertex, 6)

# Add the neighbours for a vertex.
a_vertex.neighbours.extend([a_b_edge, a_c_edge])

# Add the neighbours for b vertex.
b_vertex.neighbours.extend([a_b_edge, b_c_edge, b_d_edge, b_e_edge])

# Add the neighbours for c vertex.
c_vertex.neighbours.extend([a_c_edge, b_c_edge, c_e_edge])

# Add the neighbours for d vertex.
d_vertex.neighbours.extend([b_d_edge, d_e_edge, d_f_edge])

# Add the neighbours for e vertex.
e_vertex.neighbours.extend([b_e_edge, d_e_edge, c_e_edge, e_f_edge])

# Add the neighbours for e vertex.
f_vertex.neighbours.extend([d_f_edge, e_f_edge])


dijkstra = DijkstraAlgorithm()

shortest_path, min_distance = dijkstra.calculate_shortest_path(
	a_vertex, f_vertex
)

print(
	"Shortest path: " + shortest_path
	+ "\n"
	+ "Min Distance: " + str(min_distance)
)

assert min_distance == 14, (
	"Min Distance for this example supposed to be from A to F and to be 14."
)
