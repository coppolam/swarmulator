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
        self.im_size = self.arena_size =  int(np.abs(self.env_min) + np.abs(self.env_min)+1)
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

    def clean_cnts(self,cnts):
        bare_cnts = [row[0] for row in cnts]
        remove_indices = []
        # print(np.array(bare_cnts))
        for i, cnt in enumerate(bare_cnts):
            empty_cnt = []
            if i>0:
                for pnt in cnt:
                    if pnt not in np.concatenate(bare_cnts[:i]).ravel().reshape((-1,2)):
                        empty_cnt.append(pnt)

                if not empty_cnt:
                    remove_indices.append(i)
                    print(i)
                else:
                    cnts[i][0] = np.array(empty_cnt)             
                    cnts[i][1] = cv2.contourArea(cnts[i][0])
        
        cnts = [i for j, i in enumerate(cnts) if j not in remove_indices]

        return sorted(cnts,key=lambda x: x[1],reverse=True)



    def find_dungeon_edge(self):
        self.free_map = np.zeros(self.env_top_view.shape)*int(255) ## white = dangerous area
        gray = cv2.cvtColor(self.env_top_view, cv2.COLOR_BGR2GRAY)
        thresh = cv2.threshold(gray, 200, 255, cv2.THRESH_BINARY)[1]
        cnts = imutils.grab_contours(cv2.findContours(thresh, cv2.RETR_LIST,cv2.CHAIN_APPROX_SIMPLE))
        # num_cnts = np.shape(cnts)[0]
        # loop over the contours
        cnts_areas = []
        for c in cnts:
            cnts_areas.append([c,cv2.contourArea(c)])

        cnts_areas = sorted(cnts_areas,key=lambda x: x[1],reverse=True)
        # print(cnts)
        # print(cnts_areas)
        cnts_areas = self.clean_cnts(cnts_areas)
        # # print(cnts_areas[0][0])
        # print(cnts_areas[:2])
        # print(cnts_areas[2][0])
        
        cv2.drawContours(self.free_map, [cnts_areas[2][0]], -1, (0, 255, 0), 1)
        # # for c in cnts[0]:
        # #     cv2.drawContours(self.free_map, [c], -1, (0, 255, 0), -1)
        # # cv2.fillPoly(self.free_map,[cnts_areas[1][0]],[255,255,255]) #this marks the inside of the outer area white
        # # self.free_map += cv2.fillPoly(np.zeros(self.env_top_view.shape),[cnts_areas[1][0]],[0,0,0]) #this marks the inside of the outer area white

        # # cv2.fillPoly(self.free_map,[cnts_areas[-1][0]],[0,0,0]) #this marks the inside of the outer area white

        # num_cnts = np.shape(cnts)[0]
        # # if num_cnts > 1: # we only continue if the dungeon has inner shapes
        # #     for cnt in cnts_areas[1:]:
        # #         cv2.fillPoly(self.free_map,[cnt[0]],[255,255,255])

        cv2.imshow('contours',cv2.resize(self.free_map,(1000,1000)))
        cv2.imshow('original',cv2.resize(self.env_top_view,(1000,1000)))
        cv2.waitKey(10000)
        cv2.destroyAllWindows()  

if __name__ == '__main__':
    finder = SpaceFinder()  
    finder.create_image()
    finder.draw_image()
    finder.find_dungeon_edge()
    # print(finder.env_matrix)