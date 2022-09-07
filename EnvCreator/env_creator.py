from PIL import Image
import numpy as np
import matplotlib.pyplot as plt

class envCreator:
    def __init__(self,file,resolution=0.1,height=2,density=1):
        self.pngfile = file
        self.resolution = resolution
        self.height = height
        self.density = density
        self.occ = None
        self.path = None
        self.path_xy = None

    def image2occupancy(self):
        ## Create Occupancy Map from Image

        im = plt.imread(self.pngfile) #assumes file is formated "<name>.<type>""
        if len(im.shape) > 2:
            assert im.shape[2] in [3,4]
            if im.shape[2] == 4:
                im = im[:,:,:3]
            self.occ = np.where(np.sum(im,axis=-1)!=0,1,0)
        else:
            self.occ = np.where(im!=0,1,0)

        return self.occ

    def get_urdf(self):
        ## Create Occupancy Map from Image
        if self.occ is None:
            self.image2occupancy()

        ## Initialize URDF

        self.fname = "urdf/"+self.pngfile.split("/")[1].split(".")[0]+".urdf"
        
        with open(self.fname,"w") as f:
            f.write('<?xml version="1.0"?>\n')
            f.write('<robot name="{name}">\n'.format(name=self.pngfile.split(".")[0]))
            f.write('\n')

            mass = self.resolution*self.resolution*self.height*self.density

            link = self.make_base_link()
            links = ["base"]
            f.writelines(link)

            for i in range(self.occ.shape[0]):
                for j in range(self.occ.shape[1]):
                    if self.occ[i,j]:
                        links.append(str(i)+"_"+str(j))

                        link = self.make_link(i,j,mass)
                        joint = self.make_joint(links[0],links[-1],i,j)
                        
                        f.writelines(link)
                        f.writelines(joint)

            f.write('</robot>')

        return self.fname

    def make_base_link(self):
        lines = []
        lines.append('<link name="base">\n')
        lines.append('<inertial>\n')
        lines.append('<origin rpy="0 0 0" xyz="0 0 0"/>\n')
        lines.append('<mass value="0.0000001"/>\n')
        lines.append('<inertia ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="1"/>\n')
        lines.append('</inertial>\n')
        lines.append('</link>\n')
        lines.append('\n')
        return lines

    def make_link(self,i,j,mass):
        lines = []
        lines.append('<link name="{name}">\n'.format(name=str(i)+"_"+str(j)))
        lines.append('<inertial>\n')
        lines.append('<origin rpy="0 0 0" xyz="0 0 0"/>\n')
        lines.append('<mass value="{mass}"/>\n'.format(mass=mass))
        lines.append('<inertia ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="1"/>\n')
        lines.append('</inertial>\n')
        lines.append('<collision>\n')
        lines.append('<geometry>\n')
        lines.append('<box size="{x} {y} {z}"/>\n'.format(x=self.resolution,y=self.resolution,z=self.height))
        lines.append('</geometry>\n')
        lines.append('<origin rpy="0 0 0" xyz="0 0 0"/>\n')
        lines.append('</collision>\n')
        lines.append('<visual>\n')
        lines.append('<geometry>\n')
        lines.append('<box size="{x} {y} {z}"/>\n'.format(x=self.resolution,y=self.resolution,z=self.height))
        lines.append('</geometry>\n')
        lines.append('<material name="concrete">\n')
        lines.append('<color rgba="0.62 0.62 0.62 1"/>\n')
        lines.append('</material>\n')
        lines.append('<origin rpy="0 0 0" xyz="0 0 0"/>\n')
        lines.append('</visual>\n')
        lines.append('</link>\n')
        lines.append('\n')
        return lines

    def make_joint(self,base,link,i,j):
        lines = []
        lines.append('<joint name="{name}" type="fixed">\n'.format(name=str(link)+"_joint"))
        lines.append('<parent link="{name}"/>\n'.format(name=base))
        lines.append('<child link="{name}"/>\n'.format(name=link))
        lines.append('<origin rpy="0 0 0" xyz="{x} {y} {z}"/>\n'.format(x=self.resolution*(j-self.occ.shape[1]/2),y=self.resolution*(-i+self.occ.shape[0]-10),z=self.height/2))
        lines.append('</joint>\n')
        lines.append('\n')
        return lines

    def make_path_link(self,i,radius=0.1):
        lines = []
        lines.append('<link name="{name}">\n'.format(name=str(i)))
        lines.append('<inertial>\n')
        lines.append('<origin rpy="0 0 0" xyz="0 0 0"/>\n')
        lines.append('<mass value="0.0000001"/>\n')
        lines.append('<inertia ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="1"/>\n')
        lines.append('</inertial>\n')
        lines.append('<visual>\n')
        lines.append('<geometry>\n')
        lines.append('<sphere radius="{radius}"/>\n'.format(radius=radius))
        lines.append('</geometry>\n')
        lines.append('<material name="red">\n')
        lines.append('<color rgba="0.8 0 0 1"/>\n')
        lines.append('</material>\n')
        lines.append('<origin rpy="0 0 0" xyz="0 0 0"/>\n')
        lines.append('</visual>\n')
        lines.append('</link>\n')
        lines.append('\n')
        return lines

    def make_path_joint(self,base,link,pt,height=1):
        lines = []
        lines.append('<joint name="{name}" type="fixed">\n'.format(name=str(link)+"_joint"))
        lines.append('<parent link="{name}"/>\n'.format(name=base))
        lines.append('<child link="{name}"/>\n'.format(name=link))
        lines.append('<origin rpy="0 0 0" xyz="{x} {y} {z}"/>\n'.format(x=pt[0],y=pt[1],z=height))
        lines.append('</joint>\n')
        lines.append('\n')
        return lines

    def get_path(self,start,target,filter_dist=None,zero_loc=None):
        # if no zero location provided, assumes (0,0) in world coordinates 
        # is 1 meter above the x axis aligned to the env:
        # 10001
        # 10001
        # 10001
        # 10X01
        # 11111
        # etc.
        if self.occ is None:
            self.image2occupancy()

        # convert from xy to rc coords
        if zero_loc is None:
            zero_loc = (self.occ.shape[0]-int(1/self.resolution),self.occ.shape[1]//2+1) #in (r,c) coords
        start_rc = (-int(start[1]/self.resolution) + zero_loc[0],int(start[0]/self.resolution) + zero_loc[1])
        target_rc = (-int(target[1]/self.resolution) + zero_loc[0],int(target[0]/self.resolution) + zero_loc[1])

        # check out of bounds coords
        if start_rc[0] < 0 or start_rc[0] > self.occ.shape[0] - 1:
            print("Invalid Start Position:")
            print(start)
            return [start] 
        elif start_rc[1] < 0 or start_rc[1] > self.occ.shape[1] - 1:
            print("Invalid Start Position:")
            print(start)
            return [start] 
        elif target_rc[0] < 0 or target_rc[0] > self.occ.shape[0] - 1:
            print("Invalid Target Position:")
            print(target)
            return [target] 
        elif target_rc[1] < 0 or target_rc[1] > self.occ.shape[1] - 1:
            print("Invalid Target Position:")
            print(target)
            return [target] 

        grid = Grid(self.occ)
        start_node = Node()
        start_node.set_position(start_rc)
        target_node = Node()
        target_node.set_position(target_rc)
        self.path = A_Star_func(grid,start_node,target_node)
        if filter_dist is not None:
            self.path = self.filter_path(filter_dist)
        self.path_xy = [(self.resolution*(p[1] - zero_loc[1]),self.resolution*(-p[0] + zero_loc[0])) for p in self.path]
        self.path_xy = [tuple([int(10*p[i])/10 for i in range(len(p))]) for p in self.path_xy] #truncates decimal to 1 digit
        return self.path_xy

    def filter_path(self,dist=1):
        filtered_path = []
        loc = self.path[0]
        for p in self.path[1:]:
            if ((loc[0]-p[0])**2+(loc[1]-p[1])**2)**.5 > dist/self.resolution:
                filtered_path.append(p)
                loc = p
        if filtered_path[-1] != self.path[-1]:
            filtered_path.append(self.path[-1])
        return filtered_path

    def path2urdf(self,path=[(0,0)]):
        #path is list of tuples of waypts

        if path == [(0,0)] and self.path_xy is not None:
            path = self.path_xy

        #make base link
        self.path_fname = "urdf/path.urdf"

        with open(self.path_fname,"w") as f:
            f.write('<?xml version="1.0"?>\n')
            f.write('<robot name="path">\n')
            f.write('\n')

            link = self.make_base_link()
            links = ["base"]
            f.writelines(link)

            for i in range(len(path)-1):
                links.append(str(i))

                link = self.make_path_link(i)
                joint = self.make_path_joint(links[0],links[-1],path[i])
                        
                f.writelines(link)
                f.writelines(joint)

            links.append("target")

            link = self.make_path_link("target",radius=0.1)
            joint = self.make_path_joint(links[0],links[-1],path[-1])
                        
            f.writelines(link)
            f.writelines(joint)
            
            f.write('</robot>')

        return self.path_fname

class Node:
    def __init__(self):
        self.position = (0, 0)
        self.parent = None

        self.g = 0
        self.h = 0
        self.f = 0

    def set_position(self,pose):
        self.position = pose

    def get_position(self):
        return self.position

    def set_parent(self,parent):
        self.parent = parent

    def get_parent(self):
        return self.parent

    def set_g(self,g):
        self.g = g

    def get_g(self):
        return self.g

    def set_h(self,h):
        self.h = h

    def get_h(self):
        return self.h

    def set_f(self,f):
        self.f = f

    def get_f(self):
        return self.f

    def same_pose(self,node):
        return self.position == node.get_position()

    def __hash__(self):
        return hash((self.position,self.parent))

    def __eq__(self,node):
        if self.position == node.get_position() and self.parent == node.get_parent():
            return True
        return False

    def __repr__(self):
        return "Position: {pose}".format(pose=self.position)
        #return "(Position: {pose} , Parent: {parent})".format(pose=self.position, parent=self.parent)

class Grid:
    def __init__(self,grid):
        self.grid = grid
        self.rlim,self.clim = self.grid.shape
        self.neighbors =  [
                (-1, -1),
                (-1, 0),
                (-1, 1),
                (0, -1),
                (0, 1),
                (1, -1),
                (1, 0),
                (1, 1),
                ]

        self.neighbor_dict = {
            (-1, -1) : [(-1,-2),(-1,-3),(-1,-4),(-1,0),(-1,1),(-1,2),(-2,-1),(-2,-2),(-2,-3),(-2,-4),(-2,0),(-2,1),(-2,2),(-3,-1),(-3,-2),(-3,-3),(-3,0),(-3,1),(-4,0),(-4,-1),(-4,-2)],
            (-1, 0)  : [(-1,-1),(-1,-2),(-1,-3),(-1,1),(-1,2),(-1,3),(-2,0),(-2,-1),(-2,-2),(-2,-3),(-2,1),(-2,2),(-2,3),(-3,0),(-3,-1),(-3,-2),(-3,-3),(-3,1),(-3,2),(-3,3),(-4,0),(-4,-1),(-4,-2),(-4,-3),(-4,1),(-4,2),(-4,3)],
            (-1, 1)  : [(-1,-2),(-1,-1),(-1,0),(-1,2),(-1,3),(-1,4),(-2,-2),(-2,-1),(-2,0),(-2,1),(-2,2),(-2,3),(-2,4),(-3,-1),(-3,0),(-3,1),(-3,2),(-3,3),(-4,0),(-4,1),(-4,2)],
            (0, -1)  : [(-3,-1),(-2,-1),(-1,-1),(1,-1),(2,-1),(3,-1),(0,-2),(-3,-2),(-2,-2),(-1,-2),(1,-2),(2,-2),(3,-2),(0,-3),(-3,-3),(-2,-3),(-1,-3),(1,-3),(2,-3),(3,-3),(0,-4),(-3,-4),(-2,-4),(-1,-4),(1,-4),(2,-4),(3,-4)],
            (0, 1)   : [(-3,1),(-2,1),(-1,1),(1,1),(2,1),(3,1),(0,2),(-3,2),(-2,2),(-1,2),(1,2),(2,2),(3,2),(0,3),(-3,3),(-2,3),(-1,3),(1,3),(2,3),(3,3),(0,4),(-3,4),(-2,4),(-1,4),(1,4),(2,4),(3,4)],
            (1, -1)  : [(1,-2),(1,-3),(1,-4),(1,0),(1,1),(1,2),(2,-1),(2,-2),(2,-3),(2,-4),(2,0),(2,1),(2,2),(3,-1),(3,-2),(3,-3),(3,0),(3,1),(4,0),(4,-1),(4,-2)],
            (1, 0)   : [(1,-1),(1,-2),(1,-3),(1,1),(1,2),(1,3),(2,0),(2,-1),(2,-2),(2,-3),(2,1),(2,2),(2,3),(3,0),(3,-1),(3,-2),(3,-3),(3,1),(3,2),(3,3),(4,0),(4,-1),(4,-2),(4,-3),(4,1),(4,2),(4,3)],
            (1, 1)   : [(1,-2),(1,-1),(1,0),(1,2),(1,3),(1,4),(2,-2),(2,-1),(2,0),(2,1),(2,2),(2,3),(2,4),(3,-1),(3,0),(3,1),(3,2),(3,3),(4,0),(4,1),(4,2)],
        }      

    def get_value(self,i,j):
        return self.grid[i,j]

    def get_shape(self):
        return self.grid.shape

    def get_adjacent(self,node):
        r_,c_ = node.get_position()
        adjacent = []
        for nn in self.neighbors:
            flag = True
            r = r_ + nn[0]
            c = c_ + nn[1]
            if 0 <= r < self.rlim and 0 <= c < self.clim:
                if not self.get_value(r,c):
                    for n_ in self.neighbor_dict[nn]:
                        r_temp = r_ + n_[0]
                        c_temp = c_ + n_[1]
                        if 0 > r_temp  or r_temp > self.rlim or 0 > c_temp or c_temp > self.clim:
                            flag = False
                            break
                        elif self.get_value(r_temp,c_temp):
                            flag = False
                            break          
                else:
                    flag = False
            else:
                flag = False

            if flag:
                n = Node()
                n.set_position((r,c))
                n.set_parent(node)
                adjacent.append(n)
        return adjacent

    def is_open(self,pose):
        return not bool(self.get_value(*pose))

    def __repr__(self):
        return "{}".format(self.grid)

def A_Star_func(grid,start,target):

    if not grid.is_open(start.get_position()) or not grid.get_adjacent(start):
        print("Invalid Start Position:")
        print(start)
        return []
    elif not grid.is_open(target.get_position()) or not grid.get_adjacent(target):
        print("Invalid Target:")
        print(target)
        return [start.get_position()]

    _open = []
    _closed = []
    _open.append(start)

    max_iters = grid.get_shape()[0] * grid.get_shape()[1] 
    cnt = 0

    while _open and cnt < max_iters:
        cnt += 1

        min_f = np.argmin([n.get_f() for n in _open])
        current = _open.pop(min_f)
        if current.get_position() in _closed:
            continue
        _closed.append(current.get_position())
        if current.same_pose(target):
            print("Solution found.")
            break
        neighbors = grid.get_adjacent(current)
        for n in neighbors:
            if n.get_position() in _closed:
                continue
            n.set_g(current.get_g() + 1)
            x1, y1 = n.get_position()
            x2, y2 = target.get_position()
            n.set_h((y2 - y1) ** 2 + (x2 - x1) ** 2)
            n.set_f(n.get_h() + n.get_g())
            _open.append(n)
    else:
        print("Solution not found.")
        return [start.get_position()]
    path = []
    while current.get_parent() is not None:
        path.append(current.get_position())
        current = current.get_parent()
    path.append(current.get_position())
    return path[::-1]

if __name__ == "__main__":
    file = "occ/easy_maze.png"
    env_c = envCreator(file)
    urdf = env_c.get_urdf()
    path = env_c.get_path((0,0),(0,18),0.2)
    path_urdf = env_c.path2urdf()
    print(path)

