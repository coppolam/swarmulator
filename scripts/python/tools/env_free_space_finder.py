import random, sys, pickle, argparse, cv2, imutils
import numpy as np


#####################
#  Argument parser  #
#####################
parser = argparse.ArgumentParser(description='Find the spawnable locations within a (generated) environment with opencv')
parser.add_argument('env_name', type=str, help="(str) Folder name in /conf/environments with 'walls.txt' that is used as input file")
args = parser.parse_args()


class SpaceFinder:
    def __init__(self):
        self.walls_file = "conf/environments/" + args.env_name + "/walls.txt"
        self.env_matrix = np.loadtxt(self.walls_file)

    def create_image(self):
        self.env_min, self.env_max = np.min(self.env_matrix), np.max(self.env_matrix)
        self.arena_size =  int(np.abs(self.env_min) + np.abs(self.env_min)+1)
        self.im_size = 1000
        self.env_top_view = np.zeros((self.im_size,self.im_size,3),np.uint8)

    ## converts word coordinates to image coordinates
    def world2opencv_coords(self,x,y):
        return(int(self.im_size/2.+(x/self.arena_size)*self.im_size),int(self.im_size/2.-(y/self.arena_size)*self.im_size))

    def draw_line(self,row):
        num_segments = int(len(row)/2-1)
        for i in range(num_segments):
            start_point = self.world2opencv_coords(row[i*2],row[i*2+1])
            end_point = self.world2opencv_coords(row[i*2+2],row[i*2+3])
            self.env_top_view = cv2.line(self.env_top_view, start_point, end_point,(255,255,255),1)

    def draw_image(self):
        for line in self.env_matrix:
            self.draw_line(line)
        
        self.env_top_view = cv2.resize(self.env_top_view,(1000,1000),interpolation = cv2.INTER_AREA) # This is to make sure a wall is small compared to the total arena

    def find_dungeon_edge(self):
        self.free_map = np.ones(self.env_top_view.shape)*int(255) ## white = dangerous area
        gray = cv2.cvtColor(self.env_top_view, cv2.COLOR_BGR2GRAY)
        thresh = cv2.threshold(gray, 200, 255, cv2.THRESH_BINARY)[1]
        cnts = imutils.grab_contours(cv2.findContours(thresh, cv2.RETR_LIST,cv2.CHAIN_APPROX_SIMPLE))
        # num_cnts = np.shape(cnts)[0]
        # loop over the contours
        cnts_areas = []
        for c in cnts:
            cnts_areas.append([c,cv2.contourArea(c)])

        cnts_areas = sorted(cnts_areas,key=lambda x: x[1],reverse=True)      
        cv2.drawContours(self.free_map, [cnts_areas[1][0]], -1, (0, 0, 0), -1)    # make largest internal area a safe zone

        # find all smaller internal spaces and make them no-go zones.
        for i in range(2,np.shape(cnts_areas)[0]):
            cv2.drawContours(self.free_map, [cnts_areas[i][0]], -1, (255, 255, 255), -1)

        cv2.imshow('contours',cv2.resize(self.free_map,(1000,1000),interpolation = cv2.INTER_AREA))
        cv2.imshow('original',cv2.resize(self.env_top_view,(1000,1000),interpolation = cv2.INTER_AREA))
        cv2.waitKey(0)
        cv2.destroyAllWindows()  

if __name__ == '__main__':
    finder = SpaceFinder()  
    finder.create_image()
    finder.draw_image()
    finder.find_dungeon_edge()
    # print(finder.env_matrix)