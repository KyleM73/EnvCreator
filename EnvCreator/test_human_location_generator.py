import EnvCreator

#occupancy grid
file = "occ/easy_maze.png"

#create urdf of environment
env_c = EnvCreator.humanEnvCreator(file) ##see code for options

#get human locations
start = (0,0)
target = (0,18)
num_humans = 10
n = 10 
loc_data = env_c.generate_loc_data(start,target,num_humans,n=n,random_num=True) ##see code for options