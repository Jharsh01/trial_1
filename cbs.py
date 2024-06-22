"""

Python implementation of Conflict-based search

author: Ashwin Bose (@atb033)

"""

import sys
sys.path.insert(0, '../')
import argparse
import yaml
from math import fabs
from itertools import combinations
from copy import deepcopy
import numpy as np
import csv
import math
from a_star import AStar
import numpy as np

class Location(object):
    def __init__(self, x=-1, y=-1):
        self.x = x
        self.y = y
    def __eq__(self, other):
        return self.x == other.x and self.y == other.y
    def __str__(self):
        return str((self.x, self.y))

class State(object):
    def __init__(self, time, location,startime):
        self.time = time
        self.location = location
        self.startime = startime
    def __eq__(self, other):
        return self.time == other.time and self.location == other.location
    def __hash__(self):
        return hash(str(self.time)+str(self.location.x) + str(self.location.y))
    def is_equal_except_time(self, state):
        return self.location == state.location
    def __str__(self):
        return str((self.time, self.location.x, self.location.y))

class Conflict(object):
    VERTEX = 1
    EDGE = 2
    PASSOVER = 3
    TO_CLOSE = 4
    def __init__(self):
        self.time = -1
        self.type = -1
        self.time_1b = -1

        self.agent_1 = ''
        self.agent_2 = ''

        self.location_1 = Location()
        self.location_2 = Location()

    def __str__(self):
        return '(' + str(self.time) + ', ' + self.agent_1 + ', ' + self.agent_2 + \
             ', '+ str(self.location_1) + ', ' + str(self.location_2) + ')'

class VertexConstraint(object):
    def __init__(self, time, location):
        self.time = time
        self.location = location

    def __eq__(self, other):

        return self.time == other.time and self.location == other.location
    def __hash__(self):
        return hash(str(self.time)+str(self.location))
    def __str__(self):
        return '(' + str(self.time) + ', '+ str(self.location) + ')'

class EdgeConstraint(object):
    def __init__(self, time, location_1, location_2):
        self.time = time
        self.location_1 = location_1
        self.location_2 = location_2
    def __eq__(self, other):
        return self.time == other.time and self.location_1 == other.location_1 \
            and self.location_2 == other.location_2
    def __hash__(self):
        return hash(str(self.time) + str(self.location_1) + str(self.location_2))
    def __str__(self):
        return '(' + str(self.time) + ', '+ str(self.location_1) +', '+ str(self.location_2) + ')'
#class 

class Constraints(object):
    def __init__(self):
        self.vertex_constraints = set()
        self.edge_constraints = set()

    def add_constraint(self, other):
        self.vertex_constraints |= other.vertex_constraints
        self.edge_constraints |= other.edge_constraints

    def __str__(self):
        return "VC: " + str([str(vc) for vc in self.vertex_constraints])  + \
            "EC: " + str([str(ec) for ec in self.edge_constraints])

class Environment(object):
    def __init__(self, dimension, agents, graph, nodes):
        self.dimension = dimension
        self.nodes = nodes
        self.graph = graph
        self.agents = agents
        self.agent_dict = {}

        self.make_agent_dict()

        self.constraints = Constraints()
        self.constraint_dict = {}
        self.edge_info = {}
        
        
        self.a_star = AStar(self)


    
        

    def get_neighbors(self, state):
        neighbors = []
        n = State(state.time + 1, state.location,state.startime)
        if self.state_valid(n):
            neighbors.append(n)
            try:
                #print(self.nodes.shape)
                index = np.where(np.all(self.nodes == [n.location.x,n.location.y],axis =1))[0][0]
                for options in np.nonzero(self.graph[index])[0]:
                    #print("options",options)
                    #print("node",self.nodes[options])
                    sin = self.nodes[options][1]-state.location.y
                    cos = self.nodes[options][0]-state.location.x
                    hyp = np.sqrt((self.nodes[options][0]-state.location.x)**2 + (self.nodes[options][1]-state.location.y)**2)
                    dist = float(self.graph[index][options])
                    #print("dist",dist,"hpy",hyp)
                    x1 = round(self.nodes[options][0]- (cos/hyp)*(dist)*0.2,2)
                    y1 = round(self.nodes[options][1] - (sin/hyp)*(dist)*0.2,2)
                    x = round(state.location.x + (cos/hyp)*(dist)*0.2,2)
                    y = round(state.location.y + (sin/hyp)*(dist)*0.2,2)
                    #print(int(x+y)*100)
                    if int(self.graph[index][options]) > 1000:
                        self.edge_info[(x,y)] = [self.nodes[options][0],self.nodes[options][1],x1,y1,options,index,"entry"]
                        k = State(round( state.time + float(0.002*self.graph[index][options]),2), Location(x,y),state.startime)            
                        if self.state_valid(k) and self.transition_valid(state, k) and k.time > k.startime:
                            neighbors.append(k)
                    else :
                        k = State(state.time + float(self.graph[index][options]*0.01), Location(self.nodes[options][0],self.nodes[options][1]),state.startime)            
                        if self.state_valid(k) and self.transition_valid(state, k) and k.time > k.startime:
                            neighbors.append(k)

                        #print("uploaded index",options,"coodinates",self.nodes[options][0],self.nodes[options][1])

            except:
                #print(self.edge_info.keys())
                #print(state.location.x,state.location.y)
                if self.edge_info[(state.location.x,state.location.y)][6] == "entry":
                    x = self.edge_info[(state.location.x,state.location.y)][2]
                    y = self.edge_info[(state.location.x,state.location.y)][3]
                    x1 = self.edge_info[(state.location.x,state.location.y)][0]
                    y1 = self.edge_info[(state.location.x,state.location.y)][1]
                    options = self.edge_info[(state.location.x,state.location.y)][4]
                    index = self.edge_info[(state.location.x,state.location.y)][5]
                    k = State(round( state.time + float(self.graph[index][options])*0.006,1), Location(x,y),state.startime)            
                    if self.state_valid(k) and self.transition_valid(state, k) and k.time > k.startime:
                            neighbors.append(k)
                    self.edge_info[(x,y)] = [x1,y1,options,index,"exit",None,None]
                    #print((x,y))            
                #time = self.edge_info[state.location.x+state.location.y][2]
                #self.edge_info[x+y] = [self.edge_info[state.location.x+state.location.y][0],self.edge_info[state.location.x+state.location.y][0],time]
                elif self.edge_info[(state.location.x,state.location.y)][4]=="exit":
                    x = self.edge_info[(state.location.x,state.location.y)][0]
                    y = self.edge_info[(state.location.x,state.location.y)][1]
                    options = self.edge_info[(state.location.x,state.location.y)][2]
                    index = self.edge_info[(state.location.x,state.location.y)][3]
                
                    k = State(round( state.time + float(self.graph[index][options])*0.002,1), Location(x,y),state.startime)            
                    if self.state_valid(k) and self.transition_valid(state, k) and k.time > k.startime:
                        neighbors.append(k)
                neighbors.append(k)
        return neighbors
    def dist (self,state1,state2):
        x_diff = state1.location.x - state2.location.x
        y_diff = state1.location.y - state2.location.y
        return np.sqrt(x_diff**2 + y_diff**2)


    def get_first_conflict(self, solution):
        max_t = max([len(plan) for plan in solution.values()])
        #print(max_t)
        result = Conflict()
        for t in range(max_t):
            for k in range(max_t):
                for agent_1, agent_2 in combinations(solution.keys(), 2):
                    state_1 = self.get_state(agent_1, solution, t)
                    state_2 = self.get_state(agent_2, solution, k)
                    #print("1 time",state_1.time,"2 time",state_2.time)
                    if state_1.startime > state_1.time  or state_2.startime > state_2.time:
                        continue 
                    if state_1 == state_2:
                        result.time = state_1.time
                        result.type = Conflict.VERTEX
                        result.location_1 = state_1.location
                        result.agent_1 = agent_1
                        result.agent_2 = agent_2
                        return result

                for agent_1, agent_2 in combinations(solution.keys(), 2):
                    state_1a = self.get_state(agent_1, solution, t)
                    state_1b = self.get_state(agent_1, solution, t+1)

                    state_2a = self.get_state(agent_2, solution, k)
                    state_2b = self.get_state(agent_2, solution, k+1)
                    if state_1a.startime > state_1a.time  or state_2a.startime > state_2a.time or state_1b.startime > state_1b.time  or state_2b.startime > state_2b.time:
                        continue 

                    if state_1a==state_2b and state_1b==state_2a:
                        result.time = state_2a.time
                        result.type = Conflict.EDGE
                        result.agent_1 = agent_1
                        result.agent_2 = agent_2
                        result.location_1 = state_1a.location
                        result.location_2 = state_1b.location
                        return result
                for agent_1, agent_2 in combinations(solution.keys(), 2):
                    state_1a = self.get_state(agent_1, solution, t)
                    state_1b = self.get_state(agent_1, solution, t+1)

                    state_2a = self.get_state(agent_2, solution, k)
                    state_2b = self.get_state(agent_2, solution, k+1)
                    if state_1a.startime > state_1a.time  or state_2a.startime > state_2a.time or state_1b.startime > state_1b.time  or state_2b.startime > state_2b.time:
                        continue

                    if state_1a.time < state_2a.time and state_2a.time < state_1b.time and state_1a.location==state_2b.location and state_1b.location==state_2a.location :
                    
                        
                        result.time = state_1a.time
                        result.time_1b = state_2a.time
                        result.type = Conflict.PASSOVER
                        result.agent_1 = agent_1
                        result.agent_2 = agent_2
                        result.location_1 = state_1a.location
                        result.location_2 = state_1b.location
                        return result
                    
                for agent_1, agent_2 in combinations(solution.keys(), 2):
                    state_1 = self.get_state(agent_1, solution, t)
                    state_2 = self.get_state(agent_2, solution, k)
                    #print("1 time",state_1.time,"2 time",state_2.time)
                    if state_1.startime > state_1.time  or state_2.startime > state_2.time:
                        continue 
                    if self.dist(state_1,state_2) < 0.2 and np.abs(state_1.time - state_2.time) < 2:
                        print("new type")
                        if state_1.time < state_2.time:
                            result.time = state_1.time
                            result.time_1b = state_2.time
                            result.type = Conflict.TO_CLOSE
                            result.location_1 = state_1.location
                            result.agent_1 = agent_1
                            result.agent_2 = agent_2
                            return result
                        else :
                            result.time = state_2.time
                            result.time_1b = state_1.time
                            result.type = Conflict.TO_CLOSE
                            result.location_1 = state_1.location
                            result.agent_1 = agent_2
                            result.agent_2 = agent_1
                            return result
                    
        return False

    def create_constraints_from_conflict(self, conflict):
        constraint_dict = {}
        if conflict.type == Conflict.VERTEX:
            #print("touchedv")
            v_constraint = VertexConstraint(conflict.time, conflict.location_1)
            constraint = Constraints()
            constraint.vertex_constraints |= {v_constraint}
            constraint_dict[conflict.agent_1] = constraint
            constraint_dict[conflict.agent_2] = constraint

        elif conflict.type == Conflict.EDGE:
            #print("touchede")
            constraint1 = Constraints()
            constraint2 = Constraints()

            e_constraint1 = EdgeConstraint(conflict.time, conflict.location_1, conflict.location_2)
            e_constraint2 = EdgeConstraint(conflict.time, conflict.location_2, conflict.location_1)

            constraint1.edge_constraints |= {e_constraint1}
            constraint2.edge_constraints |= {e_constraint2}

            constraint_dict[conflict.agent_1] = constraint1
            constraint_dict[conflict.agent_2] = constraint2

        elif conflict.type == Conflict.PASSOVER:
            constraint1 = Constraints()
            constraint2 = Constraints()
            #print("touched")

            #e_constraint1 = EdgeConstraint(conflict.time, conflict.location_1, conflict.location_2)
            e_constraint2 = EdgeConstraint(conflict.time_1b, conflict.location_2, conflict.location_1)

            #constraint1.edge_constraints |= {e_constraint1}
            constraint2.edge_constraints |= {e_constraint2}

            #constraint_dict[conflict.agent_1] = constraint1
            constraint_dict[conflict.agent_2] = constraint2
            #print(str(e_constraint2))
        if conflict.type == Conflict.TO_CLOSE:
            #print("touchedv")
            v_constraint = VertexConstraint(conflict.time_1b, conflict.location_1)
            constraint = Constraints()
            constraint.vertex_constraints |= {v_constraint}
            #constraint_dict[conflict.agent_1] = constraint
            constraint_dict[conflict.agent_2] = constraint


        return constraint_dict

    def get_state(self, agent_name, solution, t):
        if t > 30:
            self.generate_plan(solution)
            exit()
        
        if t < len(solution[agent_name]):
            return solution[agent_name][t]
        else:
            return solution[agent_name][-1]

    def state_valid(self, state):
        
        return VertexConstraint(state.time, state.location) not in self.constraints.vertex_constraints #\
          #  and (state.location.x, state.location.y) not in self.obstacles 
        

    def transition_valid(self, state_1, state_2):
        return EdgeConstraint(state_1.time, state_1.location, state_2.location) not in self.constraints.edge_constraints

    def is_solution(self, agent_name):
        pass

    def admissible_heuristic(self, state, agent_name):
        goal = self.agent_dict[agent_name]["goal"]
       #print("this is a goal",goal.location.x,goal.location.y)
        return np.sqrt(np.power(fabs(state.location.x - goal.location.x),2) + np.power(fabs(state.location.y - goal.location.y),2))


    def is_at_goal(self, state, agent_name):
        goal_state = self.agent_dict[agent_name]["goal"]
        return state.is_equal_except_time(goal_state)

    def make_agent_dict(self):
        #print(self.agents)
        for agent in self.agents:
            #print(self.nodes[agent['start']][0],self.nodes[agent['start']][1])
            start_state = State(0, Location(self.nodes[agent['start']][0], self.nodes[agent['start']][1]),agent['startime'])
            goal_state = State(0, Location(self.nodes[agent['goal']][0], self.nodes[agent['goal']][1]),agent['startime'])
            

            self.agent_dict.update({agent['name']:{'start':start_state, 'goal':goal_state, 'startime':agent['startime']}})
            #print(self.agent_dict)

    def compute_solution(self):
        solution = {}
        for agent in self.agent_dict.keys():
            self.constraints = self.constraint_dict.setdefault(agent, Constraints())
            local_solution = self.a_star.search(agent)
            if not local_solution:
                return False
            solution.update({agent:local_solution})
        return solution

    def compute_solution_cost(self, solution):
        cost = 0
        for agent, path in solution.items():
            last_state = path[-1]  # Get the last state in the path
            #print(last_state)
            #print("solution_state", last_state.time)  # Assuming 'time' is a key in the state dictionary
            cost += int(last_state.time)  # Update the cost
        return cost
        #return sum([path[len(path)-1].time for path in solution.values()])
    def compute_solution_cost_final(self, solution):
        cost = 0
        for agent, path in solution.items():
            last_state = path[-1]  # Get the last state in the path
            #print(last_state)
              # Assuming 'time' is a key in the state dictionary
            cost += int(last_state['t'])  # Update the cost
            #print(cost)
        return cost

class HighLevelNode(object):
    def __init__(self):
        self.solution = {}
        self.constraint_dict = {}
        self.cost = 0

    def __eq__(self, other):
        if not isinstance(other, type(self)): return NotImplemented
        return self.solution == other.solution and self.cost == other.cost

    def __hash__(self):
        return hash((self.cost))

    def __lt__(self, other):
        return self.cost < other.cost

class CBS(object):
    def __init__(self, environment):
        self.env = environment
        self.open_set = set()
        self.closed_set = set()
        self.time = 0
    def search(self):
        start = HighLevelNode()
        # TODO: Initialize it in a better way
        start.constraint_dict = {}
        #print(self.env.agent_dict)
        for agent in self.env.agent_dict.keys():
            start.constraint_dict[agent] = Constraints()
        start.solution = self.env.compute_solution()
        if not start.solution:
            return {}
        start.cost = self.env.compute_solution_cost(start.solution)

        self.open_set |= {start}
        #print("visible",min(self.open_set))
        

        while self.open_set:
            P = min(self.open_set)
            self.open_set -= {P}
            self.closed_set |= {P}
            #print(P)

            self.env.constraint_dict = P.constraint_dict
            
            conflict_dict = self.env.get_first_conflict(P.solution)
            #print("open set",len(self.open_set),"closed set",len(self.closed_set))
            #print(conflict_dict)
            if not conflict_dict:
                print("solution found")

                return self.generate_plan(P.solution)
            #print(self.open_set)
            print("constarint found")

            constraint_dict = self.env.create_constraints_from_conflict(conflict_dict)
            print(constraint_dict)

            for agent in constraint_dict.keys():
                new_node = deepcopy(P)
                new_node.constraint_dict[agent].add_constraint(constraint_dict[agent])

                self.env.constraint_dict = new_node.constraint_dict
                new_node.solution = self.env.compute_solution()
                #print(self.generate_plan(new_node.solution))
                #print(self.generate_plan(P.solution))
                print(agent)
                #print(conflict_dict[agent])
                print("dicy",new_node.constraint_dict[agent])
                if not new_node.solution:
                    continue
                new_node.cost = self.env.compute_solution_cost(new_node.solution)
                if new_node == P:
                    print("we have a problem")

                # TODO: ending condition
                if new_node not in self.closed_set:
                    self.open_set |= {new_node}

        return {}

    def generate_plan(self, solution):
        plan = {}
        for agent, path in solution.items():
            #print("before")
            print({'t':state.time, 'x':state.location.x, 'y':state.location.y} for state in path)
            #print( "after")
            path_dict_list = [{'t':state.time, 'x':float(state.location.x), 'y':float(state.location.y)} for state in path]
            plan[agent] = path_dict_list
        # for m in self.env.constraint_dict.values():
        #     #print(str(m))
        # for agent, path in solution.items():
        #     for state in path:

                #print('t',state.time, 'x',state.location.x, 'y',state.location.y)

        return plan
def distance(lat,lon):
        lat1_rad = math.radians(33.9475402)
        lon1_rad = math.radians(-118.4276454)
        lat2_rad = math.radians(lat)
        lon2_rad = math.radians(lon)
        #print(lat,lon)
        #print(lat1_rad,lon1_rad,lat2_rad,lon2_rad)

    # Earth's radius in kilometers
        R = 63710

    # Differences in coordinates
        dlat = lat2_rad - lat1_rad
        dlon = lon2_rad - lon1_rad
        #
        # print(dlat,dlon)

    # x and y distances
        x = R * dlon * math.cos(lat1_rad)
        y = R * dlat

        return [x, y]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("param", help="input file containing map and obstacles")
    parser.add_argument("output", help="output file with the schedule")
    args = parser.parse_args()

    # Read from input file
    with open(args.param, 'r') as param_file:
        try:
            param = yaml.load(param_file, Loader=yaml.FullLoader)
        except yaml.YAMLError as exc:
            print(exc)

    dimension = param["map"]["dimensions"]
    #graph = param["Augmented_graph"]
    agents = param['agents']
    nodes = []
    #graph = np.array(graph)
    matrix = []
    with open('TestWeightedMatrix.csv', 'r') as csvfile:
        csvreader = csv.reader(csvfile)
        for row in csvreader:
            matrix.append(row)
    #print(len(matrix), len(matrix[0]))
    graph = np.array(matrix)
    graph = graph[1:, 1:]
    # if rows != cols:
    #     size = max(rows, cols)
    #     graph = np.pad(graph, ((0, size - rows), (0, size - cols)), mode='constant')
    
    with open('LabeledLAXLookupTable.csv', 'r') as csvfile:
        csvreader = csv.reader(csvfile)
        count = 0
        for row in csvreader:
            if count != 0:
                nodes.append(distance(float(row[2]),float(row[1])))
            count += 1
    #nodes = np.transpose(nodes)
    nodes = np.array(nodes)
    print(len(nodes), len(nodes[0]))
    graph = graph.astype(float)
    graph = graph/100
    print(np.min(nodes[:][1]),"MAX ",np.max(nodes[:][1]),"MIN ",np.min(nodes[:][0])," MAX",np.max(nodes[:][0]))
    with open('output.csv', 'w', newline='') as file:
        writer = csv.writer(file)
        writer.writerows(nodes)
    
    #print(nodes)
    #print(graph)
    
    #vertex_data = {node['vertex']: node['edge'] for node in nodes}

    env = Environment(dimension, agents, graph,nodes)

    # Searching
    cbs = CBS(env)
    solution = cbs.search()
    if not solution:
        print(" Solution not found" )
        return
    

    # Write to output file
    output = dict()
    output["schedule"] = solution
    #print()
    temp = env.compute_solution_cost_final(solution)
    output["cost"] = temp
    with open(args.output, 'w') as output_yaml:
        yaml.safe_dump(output, output_yaml)
    print("checking for timepass")


if __name__ == "__main__":
    main()
