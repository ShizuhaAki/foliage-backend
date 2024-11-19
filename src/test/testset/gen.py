#!/usr/bin/python3
import random
import math

def generate_graph(n, max_distance=10):
    """
    Generates a random graph with `n` nodes and random edges between them.

    Parameters:
    n (int): The number of nodes.
    max_distance (int): The maximum distance between any two nodes.

    Returns:
    str: Space-separated edge data for the graph.
    """

    def calculate_distance(node1, node2):
        """Calculate Euclidean distance between two nodes"""
        return math.sqrt((node1[0] - node2[0]) ** 2 + (node1[1] - node2[1]) ** 2)

    # Generate random coordinates for each node
    nodes = [(random.uniform(0, 100), random.uniform(0, 100)) for _ in range(n)]

    edges = []

    # Create random edges between nodes
    for i in range(n):
        for j in range(i + 1, n):
            distance = calculate_distance(nodes[i], nodes[j])

            # Add an edge with some probability
            if random.random() < 0.0005:
                edges.append(f"{i + 1} {nodes[i][0]:.2f} {nodes[i][1]:.2f} {j + 1} {nodes[j][0]:.2f} {nodes[j][1]:.2f}")

    return "\n".join(edges)

# Example usage:
n = int(input("N> "))
graph_data = generate_graph(n)
filename = input("Filename> ")
# Save to a file or print
with open(f"{filename}.txt", "x") as f:
    f.write(graph_data)

print("Random graph generated with", n, "nodes.")
