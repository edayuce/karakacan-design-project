import yaml
import heapq

class GraphPlanner:
    def __init__(self, yaml_path):
        with open(yaml_path, 'r') as f:
            self.graph_data = yaml.safe_load(f)["nodes"]

    def dijkstra(self, start_id, goal_id):
        graph = self.graph_data
        visited = set()
        min_heap = [(0, start_id, [start_id])]  # (cost, current_node, path)

        while min_heap:
            cost, current, path = heapq.heappop(min_heap)

            if current == goal_id:
                return path

            if current in visited:
                continue

            visited.add(current)
            for neighbor in graph[current]["neighbors"]:
                if neighbor not in visited:
                    new_cost = cost + self._distance(current, neighbor)
                    heapq.heappush(min_heap, (new_cost, neighbor, path + [neighbor]))

        return None  # no path found

    def _distance(self, node1, node2):
        n1 = self.graph_data[str(node1)]
        n2 = self.graph_data[str(node2)]
        dx = n1["x"] - n2["x"]
        dy = n1["y"] - n2["y"]
        return (dx ** 2 + dy ** 2) ** 0.5

