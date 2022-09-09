from PIL import Image
import numpy as np
import matplotlib.pyplot as plt
import random

from EnvCreator.env_creator import envCreator
from EnvCreator.utils import HidePrints

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
                pose = (pt[0]+dx,pt[1]+dy)
                if self.is_valid_loc(pose,start):
                    self.mark_occ(pose)
                    human_loc.append([*pose,dth])
                    break
            del path[idx]
        return human_loc

    def is_valid_loc(self,loc,start):
        loc_rc = self.xy2rc(loc,self.zero_loc)
        for i in self.check_pixels:
            r = loc_rc[0] + i[0]
            c = loc_rc[1] + i[1]
            if self.human_occ[r,c]:
                return False
        with HidePrints():
            path = self.get_path(start,loc)
        if len(path) <= 1:
            return False
        return True

    def mark_occ(self,loc):
        loc = self.xy2rc(loc,self.zero_loc)
        for i in self.check_pixels:
            r = loc[0] + i[0]
            c = loc[1] + i[1]
            self.human_occ[r,c] = 2

if __name__ == "__main__":
    file = "occ/easy_maze.png"
    env_c = humanEnvCreator(file)
    loc = env_c.get_human_locations((0,0),(0,18),2)
    print(loc)















