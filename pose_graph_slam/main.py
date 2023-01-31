import csv
import argparse
import numpy as np
np.set_printoptions(precision=4) 

import UtilsDataset as DtUtils
import matplotlib.pyplot as plt

from tqdm import tqdm
from pathlib import Path
from requests import get

from UtilsICP import *
from PoseGraphManager import *
from UtilsTransformation import *
from SonarContextManager import *

from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FFMpegWriter

parser = argparse.ArgumentParser(description='Pose Graph SLAM')

class MainParser():
    def __init__(self):
        FILE = Path(__file__).resolve()             
        ROOT = FILE.parents[1]                              
        ROOT = Path(os.path.relpath(ROOT, Path.cwd()))
        self.parser = argparse.ArgumentParser(description= "This is a initial option for pose grah slam"
                                            , fromfile_prefix_chars='@')
        self.root = ROOT
        self.parser.convert_arg_line_to_args = self.convert_arg_line_to_args
        self.initialize()
        
    def initialize(self):
        self.parser.add_argument('--sonar_angle',            type=int,   default='', help = 'sonar fov')
        self.parser.add_argument('--sonar_range',            type=int,   default='', help = 'sonar max range')
        self.parser.add_argument('--down_num_points',        type=int,   default='', help = 'mp4 name')
        self.parser.add_argument('--num_rings',              type=int,   default='', help = 'row size of patch')
        self.parser.add_argument('--num_sectors',            type=int,   default='', help = 'column size of patch')
        self.parser.add_argument('--try_gap_loop_detection', type=int,   default='', help = 'Hz of Sonar')
        self.parser.add_argument('--try_gap_pose',           type=int,   default='', help = 'Hz of Pose sensor')
        self.parser.add_argument('--col_threshold',          type=float, default='', help = 'Threshold of column shifting')
        self.parser.add_argument('--row_threshold',          type=float, default='', help = 'Threshold of row shifting')
        self.parser.add_argument('--img_dir',                type=str,   default='', help = 'Folder path of Encoded Polar Image')
        self.parser.add_argument('--pose_base_dir',          type=str,   default='', help = 'Path of Pose-csv')
        self.parser.add_argument('--odom_base_dir',          type=str,   default='', help = 'Path of Odometry-csv')
        self.parser.add_argument('--exclude_recent_nodes',   type=int,   default='', help = 'Recently visited nodes number')              
        self.parser.add_argument('--sc',                     type=str,   default='', help = 'Path of sonar context')              
        self.parser.add_argument('--sc_col',                 type=int,   default='', help = 'column size of sonar context')              
        self.parser.add_argument('--sc_row',                 type=int,   default='', help = 'row size of sonar context')              
        self.parser.add_argument('--pk',                     type=str,   default='', help = 'Path of polar key') 

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

if __name__ == '__main__':
    parser = MainParser()
    args = parser.parse()
    
    pose_manager = DtUtils.PoseDirManager(args.odom_base_dir)
    odom_data    = pose_manager.get_position(args.odom_base_dir)

    img_manager  = DtUtils.ImgDirManager(args.img_dir)
    img_paths    = img_manager.img_fullpaths
    num_imgs     = len(img_paths)    
    
    SCM = SonarContextManager(shape=[args.num_rings, args.num_sectors],
                              col_threshold=args.col_threshold,
                              row_threshold=args.row_threshold)
    
    # Pose
    # =======================================================================================================================
    PGM = PoseGraphManager()
    init_translation = odom_data[0][0:3]
    init_rotation    = odom_data[0][3:7]
    PGM.init_pose = get_transform_quat(init_rotation, init_translation)
    PGM.addPriorFactor()
    prev_pose = None
    trans = odom_data[0:3]
    quat  = odom_data[3:7]
    # =======================================================================================================================

    # Save Result
    # =======================================================================================================================
    save_dir = "result"
    if not os.path.exists(save_dir): os.makedirs(save_dir)
    ResultSaver = PoseGraphResultSaver(init_pose=PGM.curr_se3, 
                                       save_gap=300,
                                       num_frames=num_imgs,
                                       save_dir=save_dir)
    plt.style.use('dark_background')
    fig_idx = 1
    fig = plt.figure(fig_idx, figsize = (9, 6))
    writer = FFMpegWriter(fps=30)
    video_name =  str(args.down_num_points) + ".mp4"
    num_frames_to_skip_to_show = 1
    num_frames_to_save = np.floor(num_imgs/num_frames_to_skip_to_show)
    # =======================================================================================================================

    idx = 0
    for_node, loop_node = [], []
    with writer.saving(fig, video_name, num_frames_to_save):
        for for_idx, img_path in tqdm(enumerate(img_paths),total=num_imgs,mininterval=1):
            sc = np.load(args.sc.format(for_idx))
            pk = np.load(args.pk.format(for_idx))
            PGM.curr_node_idx = for_idx
            SCM.addNode(node_idx=PGM.curr_node_idx, sc=sc, pk=pk)

            if (PGM.curr_node_idx == 0):
                PGM.prev_node_idx = PGM.curr_node_idx
                prev_pose = get_transform_quat(quat[:,PGM.curr_node_idx], trans[:,PGM.curr_node_idx])
                continue
            
            # Pose Update
            curr_pose = get_transform_quat(quat[:,PGM.curr_node_idx], trans[:,PGM.curr_node_idx])
            relative_pose = get_relative_transform(curr_pose, prev_pose)
            PGM.curr_se3 = np.matmul(PGM.curr_se3, relative_pose)
            PGM.addOdometryFactor(relative_pose)
            prev_pose = curr_pose
            PGM.prev_node_idx = PGM.curr_node_idx  
            
            if(PGM.curr_node_idx > 1 and PGM.curr_node_idx % args.try_gap_loop_detection == 0):
                loop_idx, loop_dist, col_diff, row_diff = SCM.detectLoop(args.exclude_recent_nodes)
                
                if(loop_idx == None):
                    pass
                else:
                    print("Loop event detected: ", for_idx, loop_idx, loop_dist)
                    for_node.append(for_idx)
                    loop_node.append(loop_idx)
                    
                    # ICP
                    loop_transform = loop_constraint(args.img_dir, for_idx, loop_idx, col_diff, row_diff, args.sonar_angle, args.sonar_range, args.sc_col, args.sc_row)
                    PGM.addLoopFactor(loop_transform, loop_idx)
                    
                PGM.optimizePoseGraph()
                ResultSaver.saveOptimizedPoseGraphResult(PGM.curr_node_idx, PGM.graph_optimized)
            ResultSaver.saveUnoptimizedPoseGraphResult(PGM.curr_se3, PGM.curr_node_idx) 

            if(for_idx % num_frames_to_skip_to_show == 0): 
                plt.clf()
                ResultSaver.vizCurrentTrajectory(fig_idx=fig_idx)

                # plt.xlim(-70, 50)
                # plt.ylim(-50, 50)
                plt.axis('off')
                plt.draw()
                plt.pause(0.01)