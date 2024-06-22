#!/usr/bin/env python3
import yaml
import matplotlib
# matplotlib.use("Agg")
from matplotlib.patches import Circle, Rectangle, Arrow
from matplotlib.lines import Line2D
from matplotlib.collections import PatchCollection
import matplotlib.pyplot as plt
import numpy as np
from matplotlib import animation
import matplotlib.animation as manimation
import argparse
import math
import csv

Colors = ['orange', 'blue', 'green','white']
def distance(lat,lon):
        lat1_rad = math.radians(33.94)
        lon1_rad = math.radians(-118.42)
        lat2_rad = math.radians(lat)
        lon2_rad = math.radians(lon)

    # Earth's radius in kilometers
        R = 63710

    # Differences in coordinates
        dlat = lat2_rad - lat1_rad
        dlon = lon2_rad - lon1_rad

    # x and y distances
        x = R * dlon * math.cos(lat1_rad)
        y = R * dlat

        return [x, y]

class Animation:
  def __init__(self, map, schedule,graph,nodes):
    self.map = map
    self.schedule = schedule
    self.combined_schedule = {}
    self.combined_schedule.update(self.schedule["schedule"])
    self.graph =graph
    self.nodes = nodes
    aspect = map["map"]["dimensions"][0] / map["map"]["dimensions"][1]

    self.fig = plt.figure(frameon=False, figsize=(4 * aspect, 4))
    self.ax = self.fig.add_subplot(111, aspect='equal')
    self.fig.subplots_adjust(left=0,right=1,bottom=0,top=1, wspace=None, hspace=None)
    # self.ax.set_frame_on(False)
    self.lines = []
    self.patches = []
    self.artists = []
    self.agents = dict()
    self.agent_names = dict()
    # create boundary patch
    xmin = -50
    ymin = -50
    xmax = 50
    ymax = 50

    # self.ax.relim()
    plt.xlim(xmin, xmax)
    plt.ylim(ymin, ymax)
    # self.ax.set_xticks([])
    # self.ax.set_yticks([])
    # plt.axis('off')
    # self.ax.axis('tight')
    # self.ax.axis('off')
    

    self.patches.append(Rectangle((xmin, ymin), xmax - xmin, ymax - ymin, facecolor='none', edgecolor='red'))
    count = 0
    for node in self.nodes:
      #print(node['vertex'][0])
      x, y = node[0],node[1]
      self.patches.append(Circle((x, y ), 0.1, facecolor='red', edgecolor='red'))
      #print(node['vertex'],"new vertex")
      for edge in np.nonzero(self.graph[count])[0]:
        x1,y1 = self.nodes[edge][0],self.nodes[edge][1]
        print(x1,y1)
        self.lines.append(Line2D([x, x1], [y, y1], color='blue', linewidth=0.2))
        # self.ax.add_line(line)
      count +=1

    # create agents:
    self.T = 0
    # draw goals first
    for d, i in zip(map["agents"], range(0, len(map["agents"]))):

      self.patches.append(Rectangle((self.nodes[d["goal"]][0] - 0.25, self.nodes[d["goal"]][1] - 0.25), 0.5, 0.5, facecolor=Colors[0], edgecolor='black', alpha=0.5))
    for d, i in zip(map["agents"], range(0, len(map["agents"]))):
      
      name = d["name"]
      #print(name)
      self.agents[name] = Circle((self.nodes[d["start"]][0], self.nodes[d["start"]][1]), 0.3, facecolor=Colors[3], edgecolor='black')
      self.agents[name].original_face_color = Colors[0]
      self.patches.append(self.agents[name])
      self.T = max(self.T, schedule["schedule"][name][-1]["t"])
      
      self.agent_names[name] = self.ax.text(self.nodes[d["start"]][0], self.nodes[d["start"]][0], name.replace('agent', ''))
      self.agent_names[name].set_horizontalalignment('center')
      self.agent_names[name].set_verticalalignment('center')
      self.artists.append(self.agent_names[name])

    # self.ax.set_axis_off()
    # self.fig.axes[0].set_visible(False)
    # self.fig.axes.get_yaxis().set_visible(False)

    # self.fig.tight_layout()

    self.anim = animation.FuncAnimation(self.fig, self.animate_func,
                               init_func=self.init_func,
                               frames=int(self.T+1) * 10,
                               interval=100,
                               blit=True)

  def save(self, file_name, speed):
    self.anim.save(
      file_name,
      "ffmpeg",
      fps=10 * speed,
      dpi=200),
      # savefig_kwargs={"pad_inches": 0, "bbox_inches": "tight"})

  def show(self):
    plt.show()

  def init_func(self):
    for p in self.patches:
      self.ax.add_patch(p)
    for a in self.artists:
      self.ax.add_artist(a)

    for l in self.lines:
      self.ax.add_line(l)
    return self.patches + self.artists 

  def animate_func(self, i):
    for agent_name, agent in self.combined_schedule.items():
      #print(self.combined_schedule.items())
      
      #print(map["agents"])
      #print(self.combined_schedule.values())

      pos = self.getState(i / 10, agent)
      p = (pos[0], pos[1])
      self.agents[agent_name].center = p
      self.agent_names[agent_name].set_position(p)
      #print(agent)

    # reset all colors
    m = 0
    for _,agent in self.agents.items():
      if map['agents'][m]['startime'] < i/10:
        agent.set_facecolor(Colors[0])
        #print('color shift')
      m = m + 1
      
      #print(agent)

    # check drive-drive collisions
    agents_array = [agent for _,agent in self.agents.items()]
    #print(self.agents.keys())
    for k in range(0, len(agents_array)):
      for j in range(k+1, len(agents_array)):

        
        d1 = agents_array[k]
        #print(j)
        d2 = agents_array[j]
        pos1 = np.array(d1.center)
        pos2 = np.array(d2.center)
        if np.linalg.norm(pos1 - pos2) < 0.7 and map['agents'][k]['startime'] < i/10 and map['agents'][j]['startime'] < i/10:
          d1.set_facecolor('red')
          d2.set_facecolor('red')
          print("COLLISION! (agent-agent) ({}, {})".format(k, j))

    return self.patches + self.artists


  def getState(self, t, d):
    idx = 0
    while idx < len(d) and d[idx]["t"] < t:
      idx += 1
    if idx == 0:
      return np.array([float(d[0]["x"]), float(d[0]["y"])])
    elif idx < len(d):
      posLast = np.array([float(d[idx-1]["x"]), float(d[idx-1]["y"])])
      posNext = np.array([float(d[idx]["x"]), float(d[idx]["y"])])
    else:
      return np.array([float(d[-1]["x"]), float(d[-1]["y"])])
    dt = d[idx]["t"] - d[idx-1]["t"]
    t = (t - d[idx-1]["t"]) / dt
    pos = (posNext - posLast) * t + posLast
    return pos



if __name__ == "__main__":
  parser = argparse.ArgumentParser()
  parser.add_argument("map", help="input file containing map")
  parser.add_argument("schedule", help="schedule for agents")
  parser.add_argument('--video', dest='video', default=None, help="output video file (or leave empty to show on screen)")
  parser.add_argument("--speed", type=int, default=1, help="speedup-factor")
  args = parser.parse_args()


  with open(args.map) as map_file:
    map = yaml.load(map_file, Loader=yaml.FullLoader)
  nodes = []
  matrix = []
  with open('TestWeightedMatrix.csv', 'r') as csvfile:
        csvreader = csv.reader(csvfile)
        for row in csvreader:
            matrix.append(row)
    #print(len(matrix), len(matrix[0]))
  graph = np.array(matrix)
  graph = graph[1:, 1:]
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
  graph = graph / 100


  with open(args.schedule) as states_file:
    schedule = yaml.load(states_file, Loader=yaml.FullLoader)

  animation = Animation(map, schedule,graph,nodes)

  if args.video:
    animation.save(args.video, args.speed)
  else:
    animation.show()
