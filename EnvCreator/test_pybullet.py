import numpy as np
import time
import pybullet as p
import pybullet_data

import EnvCreator

#occupancy grid
file = "occ/easy_maze.png"

#create urdf of environment
env_c = EnvCreator.envCreator(file) ##see code for options
env_urdf = env_c.get_urdf()

#get A* path
path = env_c.get_path((0,0),(0,18),0.2) ##see code for options
path_urdf = env_c.path2urdf()

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

for t in range(10000):
    p.stepSimulation()
    time.sleep(1./100.)



