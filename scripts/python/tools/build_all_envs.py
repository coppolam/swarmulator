import sys, argparse
sys.path.insert(1,'../classes')
from randomize_environment import get_spawn_pos

parser = argparse.ArgumentParser(description='Build initial positions for all environments')
parser.add_argument('folder', type=str, help="(str) Folder with envs")
parser.add_argument('n_agents',type=int,help="(int) number of agents")
args = parser.parse_args()
get_spawn_pos(args.n_agents,args.folder)