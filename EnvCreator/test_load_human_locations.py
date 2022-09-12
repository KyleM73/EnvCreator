import numpy as np
import time
import pybullet as p
import pybullet_data

import EnvCreator

#occupancy grid
file = "occ/easy_maze.png"

#create urdf of environment
env_c = EnvCreator.humanEnvCreator(file) ##see code for options
env_urdf = env_c.get_urdf()

#get A* path
start = (0,0)
target = (0,18)
filter_dist = 0.2
path = env_c.get_path(start,target,filter_dist) ##see code for options
path_urdf = env_c.path2urdf()

#generate human locations
num_humans = 10
n = 4 
loc_data = env_c.generate_loc_data(start,target,num_humans,n=n,random_num=True) ##see code for options

#load human locations
from ast import literal_eval
with open("data/human_loc_data_easy_maze.txt","r") as f:
    human_locations_list = [list(literal_eval(line)) for line in f]

#init pybullet env
client = p.connect(p.GUI)
p.configureDebugVisualizer(p.COV_ENABLE_GUI,0)
p.resetSimulation()

#create ground plane
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.loadURDF("plane.urdf")

origin = [0,0]
env_range = 20+1
idx = 0

for x in range(int(n**0.5)):
    origin[0] = x*env_range
    for y in range(int(n**0.5)):
        origin[1] = y*env_range

        #load urdf files
        env_asset = p.loadURDF(env_urdf,basePosition=[origin[0],origin[1],0],useFixedBase=True)
        path_asset = p.loadURDF(path_urdf,basePosition=[origin[0],origin[1],0],useFixedBase=True)

        for h in human_locations_list[idx]:
            asset = p.loadURDF("urdf/human.urdf",[origin[0]+h[0],origin[1]+h[1],1.2],p.getQuaternionFromEuler([np.pi/2,0,h[2]]),useFixedBase=True)

        idx += 1
    
for t in range(10000):
    p.stepSimulation()
    time.sleep(1./100.)