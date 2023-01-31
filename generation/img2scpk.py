import matplotlib.pyplot as plt
import numpy as np
from PIL import Image
import os
import natsort
from tqdm import tqdm
import argparse

parser = argparse.ArgumentParser(description='Generate sonar context & polar key')

parser.add_argument('--input_img_dir', type=str, 
                    default = '../Datasets/image/')
parser.add_argument('--patch_size', type=int, 
                    default = [4, 4])
parser.add_argument('--sc_dir', type=str, 
                    default = '../Datasets/SC/')
parser.add_argument('--pk_dir', type=str, 
                    default = '../Datasets/PK/')
args = parser.parse_args()

def createDirectory(directory):
    try:
        if not os.path.exists(directory):
            os.makedirs(directory)
    except OSError:
        print("Error: Failed to create the directory.")

def ptcloud2sc(img_path, patch):
    image = np.asarray(Image.open(img_path))
    patch_row = patch[0]
    patch_col = patch[1]
    num_sector = int(image.shape[0]/patch_row)
    num_ring = int(image.shape[1]/patch_col)
    arr, sc = np.array([]), np.array([])
       
    for i in range(num_sector):
        for j in range(num_ring):
            arr = image[i*patch_row:patch_row+i*patch_row, j*patch_col:patch_col+j*patch_col]
            arr = np.array([np.amax(arr)])
            sc = np.append(sc, arr) 
    
    sc = np.reshape(sc, (num_sector, num_ring))
    
    return sc
    
def readData(imd_dir):
    img_list = os.listdir(imd_dir)
    img_list = natsort.natsorted(img_list)
    img_fullpath = [os.path.join(imd_dir, name) for name in img_list]
    return img_fullpath

def sc2pk(sc):
    return np.mean(sc, axis=1)

def main(args):
    img_list = readData(args.input_img_dir)
    num = len(img_list)

    createDirectory(args.sc_dir)
    createDirectory(args.pk_dir)

    for idx, img_path in tqdm(enumerate(img_list), total=num, mininterval=2):
        sc = ptcloud2sc(img_path, args.patch_size)
        pk = sc2pk(sc)

        np.save(args.sc_dir+'{}.npy'.format(idx), sc)
        np.save(args.pk_dir+'{}.npy'.format(idx), pk)

if __name__ == "__main__":
    main(args)