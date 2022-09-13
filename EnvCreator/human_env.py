import random

from PIL import Image
import numpy as np
import matplotlib.pyplot as plt

from EnvCreator.env_creator import envCreator
from EnvCreator.utils import HidePrints, truncate

class humanEnvCreator(envCreator):
    def __init__(self,file,resolution=0.1,height=2,density=1):
        """
        "humans" is a dictionary of groups of humans
        eg:
        one group of one human is {1:1} 
        three groups of one human and one group of two humans is {1:3,2:1}
        one group of three humans is {3:1}
        """
        super().__init__(file,resolution,height,density)
        self.human_occ = None
        
        #human model needs 50cm x 50cm of floor space
        self.check_pixels = [
            (-2,-2),(-2,-1),(-2,0),(-2,1),(-2,2),
            (-1,-2),(-1,-1),(-1,0),(-1,1),(-1,2),
            (0,-2),(0,-1),(0,0),(0,1),(0,2),
            (1,-2),(1,-1),(1,0),(1,1),(1,2),
            (2,-2),(2,-1),(2,0),(2,1),(2,2),
            ]

    def image2occupancy(self):
        super().image2occupancy()
        if self.human_occ is None:
            self.human_occ = self.occ.copy()
        return self.occ

    def get_human_locations(self,start,target,humans,zero_loc=None,xlim=1,ylim=1,start_free_zone=1,end_free_zone=0,iter_lim=100):
        if self.occ is None:
            self.image2occupancy()
        if zero_loc is None:
            self.zero_loc = (self.occ.shape[0]-int(1/self.resolution),self.occ.shape[1]//2+1) #in (r,c) coords
        else:
            self.zero_loc = zero_loc
        with HidePrints():
            path = self.get_path(start,target,0.5)
        if len(path) == 1:
            print("No path found.")
            return []
        path = path[int(start_free_zone/0.5):] #remove waypts from front
        path = path[:len(path)-int(end_free_zone/0.5)] #remove waypts from back
        rng = np.random.default_rng()
        human_loc = []
        for h in range(humans):
            if not path:
                break
            idx = random.sample(range(len(path)),1)[0]
            pt = path[idx]
            for i in range(iter_lim):
                dx,dy,dth = rng.uniform([-xlim,-ylim,-np.pi],[xlim,ylim,np.pi])
                pose = (truncate(pt[0]+dx,3),truncate(pt[1]+dy,3))
                if self.is_valid_loc(pose,path,idx):
                    self.mark_occ(pose)
                    human_loc.append([*pose,truncate(dth,3)])
                    break
            del path[idx]
        if len(human_loc) < humans:
            print(humans," humans requested but only ",len(human_loc)," valid locations found.")
        return human_loc

    def is_valid_loc(self,loc,path,idx):
        loc_rc = self.xy2rc(loc,self.zero_loc)
        if path[idx] == self.rc2xy(loc_rc,self.zero_loc):
            return True
        for i in self.check_pixels:
            r = loc_rc[0] + i[0]
            c = loc_rc[1] + i[1]
            if self.human_occ[r,c]:
                return False
        with HidePrints():
            local_path = self.get_path(path[idx],loc)
        if len(local_path) < 2:
            return False
        return True

    def mark_occ(self,loc):
        loc = self.xy2rc(loc,self.zero_loc)
        for i in self.check_pixels:
            r = loc[0] + i[0]
            c = loc[1] + i[1]
            self.human_occ[r,c] = 2

    def generate_loc_data(self,start,target,humans,zero_loc=None,xlim=1,ylim=1,start_free_zone=1,end_free_zone=0,iter_lim=100,n=10,output_dir="data/",random_num=False):
        self.loc_fname = output_dir+"human_loc_data_{env_name}.txt".format(env_name=self.pngfile.split(".")[-2].split("/")[-1])
        loc_data = []
        if random_num: #will interpret as "random num up to at most [humans]"
            rng = np.random.default_rng()
            h = rng.integers(low=1, high=humans+1, size=n)
        else:
            h = [humans for _ in range(n)]
        with open(self.loc_fname,"w") as f:
            for i in range(n):
                if i:
                    self.human_occ = self.occ.copy()
                loc = self.get_human_locations(start,target,h[i],zero_loc,xlim,ylim,start_free_zone,end_free_zone,iter_lim)
                loc_data.append(loc)
                f.writelines(str(loc)+"\n")
        return loc_data


if __name__ == "__main__":
    file = "occ/hall.png"
    env_c = humanEnvCreator(file)
    start = (0,0)
    target = (0,18)
    num_humans = 10
    loc = env_c.get_human_locations(start,target,num_humans)
    #loc_data = env_c.generate_loc_data(start,target,num_humans,random_num=True)
    print(loc)















