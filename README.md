## EnvCreator  

class for creating urdf files from occupancy grids  

also supports generating A* trajectories with collision constraints  

run `pip install -e .` to install  

test script also requires pybullet, which can be installed with `pip install pybullet`  

to create occupancy grids, use `https://www.pixilart.com/draw` or some other pixel art program and save the map as a png  

by default, 1 pixel = 10 cm x 10 cm  

run all tests from `EnvCreator/EnvCreator` folder, not from `EnvCreator/EnvCreator/test`  