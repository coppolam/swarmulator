import numpy as np
import glob

def get_spawn_pos(n_agents,folder='../../../conf/environments/'):
    files = [file for file in glob.glob(folder+'*/free_pnts.txt')]
    for file in files:
        env_folder = file.split('/')[-2]
        all_points = open(file,"r").readlines()
        selected_points_idxs = np.random.uniform(0,len(all_points),n_agents).astype(int)
        selected_points = [all_points[i] for i in selected_points_idxs]
        
        write_file = open(folder+env_folder+'/spawn_pnts.txt',"w")
        write_file.writelines(selected_points)    

        headings = np.random.uniform(-np.pi,np.pi,n_agents)
        np.savetxt(folder+env_folder+'/headings.txt',headings,delimiter=" ", fmt='%.2f')    

if __name__ == '__main__':
    get_spawn_pos(2)