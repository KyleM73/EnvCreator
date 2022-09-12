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

#get human locations
num_humans = 10
human_locations = env_c.get_human_locations(start,target,num_humans)

#init pybullet env
client = p.connect(p.GUI)
p.configureDebugVisualizer(p.COV_ENABLE_GUI,0)
p.resetSimulation()

#create ground plane
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.loadURDF("plane.urdf")

#load urdf files
env_asset = p.loadURDF(env_urdf,useFixedBase=True)
path_asset = p.loadURDF(path_urdf,useFixedBase=True)

humans_assets = []
for h in human_locations:
    asset = p.loadURDF("urdf/human.urdf",[h[0],h[1],1.2],p.getQuaternionFromEuler([np.pi/2,0,h[2]]),useFixedBase=True)
    humans_assets.append(asset)
    
for t in range(10000):
    p.stepSimulation()
    time.sleep(1./100.)



