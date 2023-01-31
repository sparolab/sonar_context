import os
import argparse
import numpy as np
np.set_printoptions(precision=4)

import matplotlib.pyplot as plt
import utils.UtilsDataset as DtUtils
import utils.UtilsVisualization as VsUtils

from tqdm import tqdm
from utils.SonarContextManager import *
from pathlib import Path

parser = argparse.ArgumentParser(description='Loop Detection Evaluation')

class MainParser():
    def __init__(self):
        FILE = Path(__file__).resolve()             
        ROOT = FILE.parents[1]                              
        ROOT = Path(os.path.relpath(ROOT, Path.cwd()))
        self.parser = argparse.ArgumentParser(description= "This is a initial option for find matching pair",
                                              fromfile_prefix_chars='@')
        self.root = ROOT
        self.parser.convert_arg_line_to_args = self.convert_arg_line_to_args
        self.initialize()
        
    def initialize(self):
        '''
        Please check the holoocean.txt
        '''
        self.parser.add_argument('--try_gap_loop_detection', type=int,   default='', help = 'Hz of Sonar')
        self.parser.add_argument('--try_gap_pose',           type=int,   default='', help = 'Hz of Pose sensor')
        self.parser.add_argument('--col_threshold',          type=float, default='', help = 'Threshold of column shifting')
        self.parser.add_argument('--row_threshold',          type=float, default='', help = 'Threshold of row shifting')
        self.parser.add_argument('--img_dir',                type=str,   default='', help = 'Folder path of Encoded Polar Image')
        self.parser.add_argument('--pose_base_dir',          type=str,   default='', help = 'Path of Pose-csv')
        self.parser.add_argument('--exclude_recent_nodes',   type=int,   default='', help = 'Recently visited nodes')              
        self.parser.add_argument('--sc',                     type=str,   default='', help = 'Path of sonar context')              
        self.parser.add_argument('--pk',                     type=str,   default='', help = 'Path of polar key')              
        self.parser.add_argument('--csvx',                   type=int,   default='', help = 'Global x in pose-csv')              
        self.parser.add_argument('--csvy',                   type=int,   default='', help = 'Global y in pose-csv')              

    def convert_arg_line_to_args(self, arg_line):
        for arg in arg_line.split():
            if not arg.strip():
                continue
            yield arg

    def parse(self):
        if sys.argv.__len__() == 2:
            arg_filename_with_prefix = '@' + sys.argv[1]
            args = self.parser.parse_args([arg_filename_with_prefix])
        else:
            args = self.parser.parse_args()
        return args

src_node = []
dst_node = []
dist_node = []

if __name__ == '__main__':
    parser = MainParser()
    args = parser.parse()
    
    img_manager = DtUtils.ImgDirManager(args.img_dir)
    img_paths = img_manager.img_fullpaths
    num_imgs = len(img_paths)

    SCM = SonarContextManager(col_threshold = args.col_threshold,
                              row_threshold = args.row_threshold)

    pose_manager = DtUtils.PoseDirManager(args.pose_base_dir)
    pose = pose_manager.getPose(args.try_gap_pose, args.csvx, args.csvy)

    src_node = []
    dst_node = []
    dist_node = []
        
    for for_idx, img_path in tqdm(enumerate(img_paths), total=num_imgs,mininterval=1):    
        sc = np.load(args.sc.format(for_idx))
        pk = np.load(args.pk.format(for_idx))
        SCM.addNode(node_idx=for_idx, sc=sc, pk=pk)

        if(for_idx > 1 and for_idx % args.try_gap_loop_detection == 0):
            loop_idx, loop_dist = SCM.detectLoop(args.exclude_recent_nodes)
            
            if(loop_idx == None):
                pass
            else:
                print("Loop event detected: ", for_idx, loop_idx, loop_dist)
                src_node.append(for_idx)
                dst_node.append(loop_idx)
                dist_node.append(loop_dist)
                
    total_loop_num = len(src_node)
    print(f"total loop num : {total_loop_num}")

    src_node = np.array(src_node)[:,None]
    dst_node = np.array(dst_node)[:,None]
    dist_node = np.array(dist_node)[:,None]

    loop = np.concatenate((src_node, dst_node, dist_node), axis=1)
    
    viz = VsUtils.VizTrajectory(loop, pose)
    viz.viz2D()
    viz.viz3D()
        
    plt.show()