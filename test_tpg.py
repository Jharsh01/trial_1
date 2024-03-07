import heapq

class Agent:
    def __init__(self, start, goal):
        self.start = start
        self.goal = goal
        self.path = []  # Path to be filled later

class Conflict:
    def __init__(self, agent1, agent2, time_step):
        self.agent1 = agent1
        self.agent2 = agent2
        self.time_step = time_step

class TemporalPlanGraph:
    def __init__(self, agents, grid_size=(5, 5)):
        self.agents = agents
        self.grid_size = grid_size
        self.graph = self.build()

    def build(self):
        graph = {time_step: {} for time_step in range(max(self.grid_size))}
        
        for agent in self.agents:
            current_pos = agent.start
            for t in range(max(self.grid_size)):
                if current_pos == agent.goal:
                    break  # Agent has reached its goal
                neighbors = self.get_valid_neighbors(current_pos)
                for neighbor in neighbors:
                    if t not in graph:
                        graph[t] = {}
                    graph[t][current_pos] = [(neighbor, 1)]  # Assume uniform cost
                current_pos = self.move_towards_goal(current_pos, agent.goal)
        return graph

    def get_valid_neighbors(self, position):
        neighbors = []
        for dx, dy in [(1, 0), (-1, 0), (0, 1), (0, -1)]:
            x, y = position[0] + dx, position[1] + dy
            if 0 <= x < self.grid_size[0] and 0 <= y < self.grid_size[1]:
                neighbors.append((x, y))
        return neighbors

    def move_towards_goal(self, current_pos, goal_pos):
        x_diff = goal_pos[0] - current_pos[0]
        y_diff = goal_pos[1] - current_pos[1]
        if x_diff != 0:
            return (current_pos[0] + (x_diff // abs(x_diff)), current_pos[1])
        elif y_diff != 0:
            return (current_pos[0], current_pos[1] + (y_diff // abs(y_diff)))
        else:
            return current_pos

def a_star_search(graph, start, goal):
    frontier = [(0, start)]
    came_from = {}
    cost_so_far = {start: 0}

    while frontier:
        current_cost, current_node = heapq.heappop(frontier)

        if current_node == goal:
            break

        for next_node, cost in graph[current_node]:
            new_cost = cost_so_far[current_node] + cost
            if next_node not in cost_so_far or new_cost < cost_so_far[next_node]:
                cost_so_far[next_node] = new_cost
                priority = new_cost + heuristic(next_node, goal)
                heapq.heappush(frontier, (priority, next_node))
                came_from[next_node] = current_node

    path = reconstruct_path(came_from, start, goal)
    return path

def heuristic(node, goal):
    # Simple Manhattan distance heuristic
    return abs(goal[0] - node[0]) + abs(goal[1] - node[1])

def reconstruct_path(came_from, start, goal):
    current = goal
    path = []
    while current != start:
        path.append(current)
        current = came_from[current]
    path.append(start)
    path.reverse()
    return path

def conflict_based_search(agents):
    tpg = TemporalPlanGraph(agents)
    tpg.build()

    conflicts = detect_conflicts(agents)
    while conflicts:
        resolve_conflicts(agents, conflicts)
        tpg.build()
        conflicts = detect_conflicts(agents)

    return [agent.path for agent in agents]

def detect_conflicts(agents):
    conflicts = []
    for i, agent1 in enumerate(agents):
        for j, agent2 in enumerate(agents):
            if i != j:
                # Example conflict detection (can be more sophisticated)
                if agent1.path and agent2.path:
                    for t in range(min(len(agent1.path), len(agent2.path))):
                        if agent1.path[t] == agent2.path[t]:
                            conflicts.append(Conflict(agent1, agent2, t))
    return conflicts

def resolve_conflicts(agents, conflicts):
    # Example conflict resolution (can be more sophisticated)
    for conflict in conflicts:
        if len(conflict.agent1.path) >= len(conflict.agent2.path):
            conflict.agent1.path = replan(conflict.agent1)
        else:
            conflict.agent2.path = replan(conflict.agent2)

def replan(agent):
    # Example replanning strategy (e.g., using A* search)
    graph = agent.tpg.graph
    start = agent.start
    goal = agent.goal
    path = a_star_search(graph, start, goal)
    return path

# Example usage:
agent1 = Agent((0, 0), (4, 4))
agent2 = Agent((2, 0), (2, 4))
agents = [agent1, agent2]

for agent in agents:
    agent.tpg = TemporalPlanGraph([agent])

paths = conflict_based_search(agents)
print(paths)
