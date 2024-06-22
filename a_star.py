"""

AStar search

author: Ashwin Bose (@atb033)

"""


import numpy as np
from math import fabs
class AStar():
    def __init__(self, env):
        self.agent_dict = env.agent_dict
        self.admissible_heuristic = env.admissible_heuristic
        self.is_at_goal = env.is_at_goal
        self.get_neighbors = env.get_neighbors
        self.graph = env.graph
        self.nodes = env.nodes
        self.edge_info = env.edge_info

    def reconstruct_path(self, came_from, current):
        total_path = [current]
        while current in came_from.keys():
            current = came_from[current]
            total_path.append(current)
        return total_path[::-1]

    def search(self, agent_name):
        """
        low level search 
        """
        initial_state = self.agent_dict[agent_name]["start"]
        time_cost = 1
        
        closed_set = set()
        open_set = {initial_state}

        came_from = {}

        g_score = {} 
        g_score[initial_state] = 0

        f_score = {} 

        f_score[initial_state] = self.admissible_heuristic(initial_state, agent_name)

        while open_set:
            temp_dict = {open_item:f_score.setdefault(open_item, float("inf")) for open_item in open_set}
            current = min(temp_dict, key=temp_dict.get)
            if agent_name=='agent0':
                print("current",current.location.x,current.location.y)

            if self.is_at_goal(current, agent_name):
                return self.reconstruct_path(came_from, current)

            open_set -= {current}
            closed_set |= {current}

            neighbor_list = self.get_neighbors(current)
            

            for neighbor in neighbor_list:
                if neighbor in closed_set:
                    continue
                #index_c = np.where(np.all(self.nodes == [current.location.x,current.location.y],axis =1))[0][0]
                #if agent_name=='agent0':
                    #print("neighbor",neighbor.location.x,neighbor.location.y)
                #print(self.nodes)
                #index_n = np.where(np.all(self.nodes == [neighbor.location.x,neighbor.location.y],axis =1))[0][0]
                if np.sqrt((current.location.x - neighbor.location.x)**2 +(current.location.y - neighbor.location.y)**2) ==0:
                    step_cost = time_cost
                else:
                    step_cost = np.sqrt((current.location.x - neighbor.location.x)**2 +(current.location.y - neighbor.location.y)**2) + time_cost
                #step_cost = fabs(current.location.x-neighbor.location.x) + fabs(current.location.y-neighbor.location.y) + 1
                

                
                tentative_g_score = g_score.setdefault(current, float("inf")) + step_cost
                #print("step_cost",step_cost,"current",index_c,"neighbour",index_n,"tentative_g",tentative_g_score,"potencial",g_score.setdefault(neighbor, float("inf")))
                

                if neighbor not in open_set:
                    open_set |= {neighbor}
                elif tentative_g_score >= g_score.setdefault(neighbor, float("inf")):
                    continue
                #print("reached here")
                
                

                came_from[neighbor] = current

                g_score[neighbor] = tentative_g_score
                f_score[neighbor] = g_score[neighbor] + self.admissible_heuristic(neighbor, agent_name)
        return False

