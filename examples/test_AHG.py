import numpy as np
import time
import pybullet as p
import pybullet_data

import EnvCreator

file="AHG_hall.png"

env_c = EnvCreator.envCreator(file)
env_urdf = env_c.get_urdf_fast()

start = (0,1)
target = (0,6)
filter_dist = 0.2
path = env_c.get_path(start,target,filter_dist)
path_urdf = env_c.path2urdf()

client = p.connect(p.GUI)
p.configureDebugVisualizer(p.COV_ENABLE_GUI,0)
p.resetSimulation()

p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.loadURDF("plane.urdf")

env_asset = p.loadURDF(env_urdf,useFixedBase=True)
path_asset = p.loadURDF(path_urdf,useFixedBase=True)

for t in range(1000):
    p.stepSimulation()
    time.sleep(1/100.)


