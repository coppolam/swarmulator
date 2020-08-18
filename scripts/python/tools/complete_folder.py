import random, sys, pickle, argparse, cv2, imutils, os, glob
import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
from itertools import groupby, cycle 

#####################
#  Argument parser  #
#####################
parser = argparse.ArgumentParser(description='Find the spawnable locations within a (generated) environment with opencv')
parser.add_argument('-env_name', type=str, help="(str) Folder name in /conf/environments with 'walls.txt' that is used as input file", default='random')
parser.add_argument('-debug_mode', type=str, help="(str) if True, showing opencv windows with selected area",default='False')
parser.add_argument('-grid_size', type=int, help="(int) represents the grid size used to locate free areas and eventually return free points",default=100)
parser.add_argument('-wall_dist', type=int, help="(int) number of cells kept to the closest wall when spawning",default=20)
args = parser.parse_args()

def groupSequence(l): 
    temp_list = cycle(l) 
  
    next(temp_list) 
    groups = groupby(l, key = lambda j: j + 1 == next(temp_list)) 
    for k, v in groups: 
        if k: 
            yield tuple(v) + (next((next(groups)[1])), ) 

class SpaceFinder:
    def __init__(self):
        self.rec_arena_size = 10
        self.env_dir = "conf/environments/" + args.env_name + "/"
        self.walls_file = self.env_dir + "walls.txt"
        self.img_files = [file for file in glob.glob(self.env_dir+'*.png')] + [file for file in glob.glob(self.env_dir+'*.jpg')] + [file for file in glob.glob(self.env_dir+'*.jpeg')]
        self.im_size = args.grid_size
        self.env_top_view = np.zeros((self.im_size,self.im_size,3),np.uint8)
    def check_walls_exist(self):
        self.walls_exist = os.path.exists(self.walls_file)
        if self.walls_exist:
            self.env_matrix = np.loadtxt(self.walls_file)
        if self.img_files:
            self.img_exists = True
            self.env_top_view = cv2.imread(self.img_files[0])

            self.im_size = np.max([np.max(np.shape(self.env_top_view)),self.im_size])
        else:
            self.img_exists = False

    def create_image(self):
        self.env_min, self.env_max = np.min(self.env_matrix), np.max(self.env_matrix)
        self.x_coords, self.y_coords = np.concatenate([line[0:-1:2] for line in self.env_matrix]).ravel().tolist(), np.concatenate([line[1:-1:2] for line in self.env_matrix]).ravel().tolist()
        self.x_min, self.x_max, self.y_min, self.y_max = np.min(self.x_coords), np.max(self.x_coords), np.min(self.y_coords), np.max(self.y_coords)
        self.x_dim, self.y_dim = self.x_max-self.x_min, self.y_max-self.y_min
        

    ## converts world coordinates to opencv coordinates
    def world2opencv_coords(self,x,y):
        return(int(np.floor((x-self.x_min)*(self.im_size/(self.x_dim)))),int(np.floor((self.y_max-y)*(self.im_size/self.y_dim))))

    ## converts image coordinates (!= opencv) to world coordinates
    def imgworld_coords(self,x,y):
        return(float(self.x_min+(y*(self.x_dim/self.im_size))),float(self.y_max-(x*(self.y_dim/self.im_size))))

    def draw_line(self,row):
        num_segments = int(len(row)/2-1)
        for i in range(num_segments):
            start_point = np.clip(self.world2opencv_coords(row[i*2],row[i*2+1]),0,self.im_size-1)
            end_point = np.clip(self.world2opencv_coords(row[i*2+2],row[i*2+3]),0,self.im_size-1)
            self.env_top_view = cv2.line(self.env_top_view, tuple(start_point), tuple(end_point),(255,255,255),1)

    def create_walls(self):
        c = self.rec_arena_size/self.im_size
        w_h = []
        # Horizontal walls
        self.tiles_level = cv2.cvtColor(self.env_top_view, cv2.COLOR_BGR2GRAY)
        for idx,row in enumerate(self.tiles_level):
            # print("***")
            data = np.where(row > 0)[0]
            if data.size > 0:
                    a = list(groupSequence(list(data)))
                    for item in a:
                        # if len(item) > 5:
                            w_h.append([min(item)*c,-idx*c,max(item)*c,-idx*c])
        # print(self.wall_coordinates)
        w_h = np.asarray(w_h)
        # Vertical walls
        w_v = []
        for idx,row in enumerate(self.tiles_level.T):
            # print("***")
            data = np.where(row > 0)[0]
            if data.size > 0:
                    # print(data)
                    a = list(groupSequence(list(data)))
                    for item in a:
                        # if len(item) >= 5:
                            w_v.append([idx*c,-min(item)*c,idx*c,-max(item)*c])
        # print(self.wall_coordinates)
        w_v = np.asarray(w_v)
        w = np.vstack((w_h,w_v))
        
        w[:,[0,2]] = (w[:,[0,2]] - w[:,[0,2]].mean()) # Center horizontally
        w[:,[1,3]] = (w[:,[1,3]] - w[:,[1,3]].mean()) # Center vertically      
		# Center vertically
        self.env_matrix  = w.tolist()


        NEWLINE_SIZE_IN_BYTES = -1  # -2 on Windows?
        with open(self.walls_file, 'wb') as fout:  # Note 'wb' instead of 'w'
            np.savetxt(fout, self.env_matrix, delimiter=" ", fmt='%.1f')
            fout.seek(NEWLINE_SIZE_IN_BYTES, 2)
            fout.truncate()

    def draw_image(self):
        self.env_top_view = np.zeros((self.im_size,self.im_size,3),np.uint8)
        for line in self.env_matrix:
            self.draw_line(line)
        self.env_top_view = cv2.resize(self.env_top_view,(self.im_size,self.im_size),interpolation = cv2.INTER_AREA) # This is to make sure a wall is small compared to the total arena
        cv2.imwrite(self.env_dir+"top_view.jpg",self.env_top_view)

    def find_dungeon_edge(self):
        self.free_map = np.ones(self.env_top_view.shape)*int(255) ## white = dangerous area
        gray = cv2.cvtColor(self.env_top_view, cv2.COLOR_BGR2GRAY)
        thresh = cv2.threshold(gray, 200, 255, cv2.THRESH_BINARY)[1]
        cnts = imutils.grab_contours(cv2.findContours(thresh, cv2.RETR_LIST,cv2.CHAIN_APPROX_SIMPLE))
        cnts_areas = []
        for c in cnts:
            cnts_areas.append([c,cv2.contourArea(c)])
        cnts_areas = sorted(cnts_areas,key=lambda x: x[1],reverse=True)   
        cv2.drawContours(self.free_map, [cnts_areas[1][0]], -1, (0, 0, 0), -1)    # make largest internal area a safe zone

        # find all smaller internal spaces and make them no-go zones.
        for i in range(2,np.shape(cnts_areas)[0]):
            cv2.drawContours(self.free_map, [cnts_areas[i][0]], -1, (255, 255, 255), -1)
        return False
    
    def write_free_points(self):
        self.free_array = []
        for i in range(args.wall_dist,self.im_size-args.wall_dist):
            for j in range(args.wall_dist,self.im_size-args.wall_dist):
                if np.all(self.free_map[i-args.wall_dist:i+args.wall_dist,j-args.wall_dist:j+args.wall_dist]==0):
                    self.free_array.append(self.imgworld_coords(i,j))
        np.savetxt(self.env_dir+"free_pnts.txt",self.free_array,delimiter=" ", fmt='%.1f')

    def debug_window(self):

        cv2.imshow('contours',cv2.resize(self.free_map,(1000,1000)))
        cv2.imshow('original',cv2.resize(self.env_top_view,(1000,1000),interpolation = cv2.INTER_AREA))
        cv2.waitKey(0)
        cv2.destroyAllWindows()  

if __name__ == '__main__':
    finder = SpaceFinder()  
    finder.check_walls_exist()
    if finder.img_exists or finder.walls_exist:
        if finder.walls_exist: 
            finder.create_image()
            finder.draw_image()
        elif finder.img_exists:
            finder.create_walls()
            finder.create_image()       
        finder.find_dungeon_edge()
        finder.write_free_points()
        
        if args.debug_mode=='True':
            finder.debug_window()
    else:
        print("Please provide a binary image or a 'walls.txt' ")