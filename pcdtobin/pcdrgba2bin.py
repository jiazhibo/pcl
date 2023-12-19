#!/usr/bin/env python
#
# Module:       pcd2bin.py
# Description:  .pcd to .bin converter
#
# Author:       Yuseung Na (ys.na0220@gmail.com)
# Version:      1.0
#
# Revision History
#       January 19, 2021: Yuseung Na, Created
#

import numpy as np
import os
import argparse
#import pypcd

from pypcd import pypcd
import csv
from tqdm import tqdm

def main():
    ## Add parser
    parser = argparse.ArgumentParser(description="Convert .pcd to .bin")
    parser.add_argument(
        "--pcd_path",
        help=".pcd file path.",
        type=str,
        default="/home/user/lidar_pcd"
    )
    parser.add_argument(
        "--bin_path",
        help=".bin file path.",
        type=str,
        default="/home/user/lidar_bin"
    )
    parser.add_argument(
        "--label_path",
        help=".label file path.",
        type=str,
        default="/home/user/lidar_label"
    )
    parser.add_argument(
        "--file_name",
        help="File name.",
        type=str,
        default="file_name"
    )
    args = parser.parse_args()

    ## Find all pcd files
    pcd_files = []
    for (path, dir, files) in os.walk(args.pcd_path):
        for filename in files:
            # print(filename)
            ext = os.path.splitext(filename)[-1]
            if ext == '.pcd':
                pcd_files.append(path + "/" + filename)

    ## Sort pcd files by file name
    pcd_files.sort()   
    print("Finish to load point clouds!")

    ## Make bin_path directory
    try:
        if not (os.path.isdir(args.bin_path)):
            os.makedirs(os.path.join(args.bin_path))
    except OSError as e:
        if e.errno != errno.EEXIST:
            print ("Failed to create directory!!!!!")
            raise

    ## Generate csv meta file
    csv_file_path = os.path.join(args.bin_path, "meta.csv")
    csv_file = open(csv_file_path, "w")
    meta_file = csv.writer(
        csv_file, delimiter=",", quotechar="|", quoting=csv.QUOTE_MINIMAL
    )
    ## Write csv meta file header
    meta_file.writerow(
        [
            "pcd file name",
            "bin file name",
        ]
    )
    print("Finish to generate csv meta file")

    ## Converting Process
    print("Converting Start!")
    seq = 0
    for pcd_file in tqdm(pcd_files):
        ## Get pcd file
        pc = pypcd.PointCloud.from_path(pcd_file)

        ## Generate bin file name
        bin_file_name = "{}_{:05d}.bin".format(args.file_name, seq)
        bin_file_path = os.path.join(args.bin_path, bin_file_name)

        label_file_name = "{}_{:05d}.label".format(args.file_name, seq)
        label_file_path = os.path.join(args.label_path, label_file_name)

        #print(bin_file_path)
        #print(label_file_name)

        ## Get data from pcd (x, y, z, intensity, ring, time)
        np_x = (np.array(pc.pc_data['x'], dtype=np.float32)).astype(np.float32)
        np_y = (np.array(pc.pc_data['y'], dtype=np.float32)).astype(np.float32)
        np_z = (np.array(pc.pc_data['z'], dtype=np.float32)).astype(np.float32)
        #np_i = (np.array(pc.pc_data['intensity'], dtype=np.float32)).astype(np.float32)/256
        #np_i = (np.array(pc.pc_data['z'], dtype=np.float32)).astype(np.float32)
        ####save must be transfer to float32
        np_i = (np.zeros(pc.pc_data['z'].shape)).astype(np.float32)
        #np_i = (np.array(pc.pc_data['intensity'], dtype=np.float32)).astype(np.float32)

        # np_r = (np.array(pc.pc_data['ring'], dtype=np.float32)).astype(np.float32)
        # np_t = (np.array(pc.pc_data['time'], dtype=np.float32)).astype(np.float32)

        #print("np_x size = ", np_x.size)
        #print("np_x  = ", np_x)
        #print("np_y size = ", np_y.size)
        #print("np_z size = ", np_z.size)


        #save label file
        np_label = (np.zeros(pc.pc_data['x'].shape)).astype(np.int32)
        np_rgb = (np.array(pc.pc_data['rgba'], dtype=np.uint32)).astype(np.uint32)
        #np_green = (np.array(pc.pc_data['g'], dtype=np.int8)).astype(np.int8)
        #np_blue = (np.array(pc.pc_data['b'], dtype=np.int8)).astype(np.int8)
        #print("np_rgb  = ", np_rgb)


        counter_smog = 0
        counter_pot = 0
        counter_cooktop = 0
        counter_hand = 0
        counter_body = 0
        counter_wall = 0
        counter_unknow = 0
        counter_other = 0
        counter_error = 0

        for i in range(np_label.size):
            if np_rgb[i] == 16711680:
                np_label[i] = 3
                counter_smog = counter_smog + 1
            elif np_rgb[i] == 65280:
                np_label[i] = 2
                counter_pot = counter_pot + 1
            elif np_rgb[i] == 255:
                np_label[i] = 1
                counter_cooktop = counter_cooktop + 1
            elif np_rgb[i] == 16711935:
                np_label[i] = 4
                counter_hand = counter_hand + 1
            elif np_rgb[i] == 16776960:
                np_label[i] = 5
                counter_body = counter_body + 1
            elif np_rgb[i] == 65535:
                np_label[i] = 0
                counter_unknow = counter_unknow + 1
            elif np_rgb[i] == 4569946:
                np_label[i] = 6
                counter_wall = counter_wall + 1
            elif np_rgb[i] == 11180373:
                np_label[i] = 7
                counter_other = counter_other + 1
            else:
                np_label[i] = 8
                counter_error = counter_error + 1

        #print("np_label  = ", np_label)
        print("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!counter_smog  = ", counter_smog)
        #print("counter_pot  = ", counter_pot)
        #print("counter_cooktop  = ", counter_cooktop)
        #print("counter_hand  = ", counter_hand)
        #print("counter_body  = ", counter_body)
        #print("counter_wall  = ", counter_wall)
        #print("counter_unknow  = ", counter_unknow)
        #print("counter_other  = ", counter_other)
        #print("counter_error  = ", counter_error)

        np_label = np_label.reshape((-1)).astype(np.int32)
        np_label.tofile(label_file_path)


        ## Stack all data    
        points_32 = np.transpose(np.vstack((np_x, np_y, np_z, np_i)))
        print("points_32 = ", points_32)
        ## Save bin file                                    
        points_32.tofile(bin_file_path)

        ## Write csv meta file
        meta_file.writerow(
            [os.path.split(pcd_file)[-1], bin_file_name]
        )

        seq = seq + 1
    
if __name__ == "__main__":
    main()
