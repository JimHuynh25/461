import csv
import heapq
import time
from math import radians, sin, cos, sqrt, atan2

# Load city coordinates from a CSV file into a dictionary
def load_city_coordinates(filename):
    city_coordinates = {}
    with open(filename, 'r') as file:
        csv_reader = csv.reader(file)
        for row in csv_reader:
            city_name, lat, lon = row
            city_coordinates[city_name] = (float(lat), float(lon))
    return city_coordinates

# Load city adjacencies from a text file into a graph representation
def load_adjacency_graph(filename):
    adjacency_graph = {}
    with open(filename, 'r') as file:
        for line in file:
            city1, city2 = line.strip().split()
            adjacency_graph.setdefault(city1, []).append(city2)
            adjacency_graph.setdefault(city2, []).append(city1)
    return adjacency_graph

# Calculate the Haversine distance between two coordinates (in kilometers)
def haversine(coord1, coord2):
    lat1, lon1 = coord1
    lat2, lon2 = coord2
    radius = 6371  # Earth's radius in km

    dlat = radians(lat2 - lat1)
    dlon = radians(lon2 - lon1)

    a = sin(dlat / 2)**2 + cos(radians(lat1)) * cos(radians(lat2)) * sin(dlon / 2)**2
    c = 2 * atan2(sqrt(a), sqrt(1 - a))

    return radius * c

# Brute-force search (Undirected)
def brute_force_search(start, end, adjacency_graph):
    visited = set()
    stack = [(start, [])]

    while stack:
        current, path = stack.pop()
        if current == end:
            return path + [current]
        if current not in visited:
            visited.add(current)
            for neighbor in adjacency_graph.get(current, []):
                if neighbor not in visited:
                    stack.append((neighbor, path + [current]))
    return None

# Breadth-first search (BFS)
def bfs(start, end, adjacency_graph):
    visited = set()
    queue = [(start, [])]

    while queue:
        current, path = queue.pop(0)
        if current == end:
            return path + [current]
        if current not in visited:
            visited.add(current)
            for neighbor in adjacency_graph.get(current, []):
                if neighbor not in visited:
                    queue.append((neighbor, path + [current]))
    return None

# Depth-first search (DFS)
def dfs(start, end, adjacency_graph):
    visited = set()
    stack = [(start, [])]

    while stack:
        current, path = stack.pop()
        if current == end:
            return path + [current]
        if current not in visited:
            visited.add(current)
            for neighbor in adjacency_graph.get(current, []):
                if neighbor not in visited:
                    stack.append((neighbor, path + [current]))
    return None

# Iterative Deepening Depth-First Search (ID-DFS)
def id_dfs(start, end, adjacency_graph):
    depth = 0
    while True:
        result = dfs_recursive(start, end, [], depth, adjacency_graph)
        if result:
            return result
        depth += 1

def dfs_recursive(current, end, path, depth, adjacency_graph):
    if current == end:
        return path + [current]
    if depth == 0:
        return None
    for neighbor in adjacency_graph.get(current, []):
        if neighbor not in path:
            result = dfs_recursive(neighbor, end, path + [current], depth - 1, adjacency_graph)
            if result:
                return result
    return None

# Best-first search (Greedy)
def best_first_search(start, end, city_coordinates, adjacency_graph):
    visited = set()
    heap = [(haversine(city_coordinates[start], city_coordinates[end]), start, [])]

    while heap:
        _, current, path = heapq.heappop(heap)
        if current == end:
            return path + [current]
        if current not in visited:
            visited.add(current)
            for neighbor in adjacency_graph.get(current, []):
                if neighbor not in visited:
                    heapq.heappush(heap, (haversine(city_coordinates[neighbor], city_coordinates[end]), neighbor, path + [current]))
    return None

# A* search
def astar_search(start, end, city_coordinates, adjacency_graph):
    visited = set()
    heap = [(0, haversine(city_coordinates[start], city_coordinates[end]), start, [])]

    while heap:
        _, _, current, path = heapq.heappop(heap)
        if current == end:
            return path + [current]
        if current not in visited:
            visited.add(current)
            for neighbor in adjacency_graph.get(current, []):
                if neighbor not in visited:
                    g = len(path) + 1  # Actual cost (from start to current node)
                    h = haversine(city_coordinates[neighbor], city_coordinates[end])  # Heuristic (distance to end)
                    f = g + h  # Total cost (f = g + h)
                    heapq.heappush(heap, (f, h, neighbor, path + [current]))
    return None

# Main program loop
def main():
    city_coordinates = load_city_coordinates('coordinates.csv')
    adjacency_graph = load_adjacency_graph('Adjacencies.txt')

    while True:
        start_city = input("Enter the starting town: ")
        end_city = input("Enter the ending town: ")

        if start_city not in city_coordinates or end_city not in city_coordinates:
            print("Invalid cities. Please enter valid city names.")
            continue

        print("Select a search method:")
        print("1. Brute-force (Undirected)")
        print("2. Breadth-first search")
        print("3. Depth-first search")
        print("4. ID-DFS search")
        print("5. Best-first search")
        print("6. A* search")

        method = int(input("Enter the method number (1-6): "))

        start_time = time.time()

        if method == 1:
            path = brute_force_search(start_city, end_city, adjacency_graph)
        elif method == 2:
            path = bfs(start_city, end_city, adjacency_graph)
        elif method == 3:
            path = dfs(start_city, end_city, adjacency_graph)
        elif method == 4:
            path = id_dfs(start_city, end_city, adjacency_graph)
        elif method == 5:
            path = best_first_search(start_city, end_city, city_coordinates, adjacency_graph)
        elif method == 6:
            path = astar_search(start_city, end_city, city_coordinates, adjacency_graph)
        else:
            print("Invalid method selection. Please enter a valid method number (1-6).")
            continue

        end_time = time.time()

        if path:
            print("Route found:", " -> ".join(path))
            total_distance = sum(haversine(city_coordinates[path[i]], city_coordinates[path[i + 1]]) for i in range(len(path) - 1))
            print("Total distance:", round(total_distance, 2), "km")
            print("Time taken:", round(end_time - start_time, 4), "seconds")
        else:
            print("No route found.")

        choice = input("Do you want to search again? (yes/no): ")
        if choice.lower() != 'yes':
            break

if __name__ == "__main__":
    main()
