import os
import csv
import numpy as np
import natsort

class ImgDirManager:
    def __init__(self, img_dir):
        self.img_dir = os.path.join(img_dir)                      
        self.imgfile_list = os.listdir(self.img_dir)                                              
        self.imgfile_list = natsort.natsorted(self.imgfile_list)                                                                   
        self.img_fullpaths = [os.path.join(self.img_dir, name) for name in self.imgfile_list]    
        self.num_imgs = len(self.imgfile_list)                                                    
        
class PoseDirManager:
    def __init__(self, pose_base_dir):
        self.pose_path = pose_base_dir
    
    def get_position(self, pose_path):
        pose_csv = open(pose_path, 'r', encoding='utf-8')
        initial_data = csv.reader(pose_csv)
        position_data = []
        
        for pose in initial_data:
            position = list(map(float, pose))
            position_data.append(position)

        position_data = np.array(position_data)
        position_data = position_data.T    
        return position_data