#!/usr/bin/env python
import rosbag
import rospy
import os
import math
import numpy as np


import datetime as dt
from pytz import timezone
import pytz
import argparse
from sensor_msgs.msg import Image
from std_msgs.msg import Header
import cv2
from cv_bridge import CvBridge, CvBridgeError

class DepthWriter:
    def __init__(self, base_path, date, drive, depth_path, depth_raw_path, name, name_raw, camera, frame, image_path='image_02', radius=5):

        self.main_dir = os.path.join(base_path, '{}/{}_drive_{:04d}_sync'.format(date,date, drive))
        self.depth_dir = os.path.join(self.main_dir, depth_path)
        self.depth_raw_dir = os.path.join(self.main_dir, depth_raw_path)
        self.timestamp_file = os.path.join(self.main_dir, '{}/timestamps.txt'.format(image_path))
        
        print (self.main_dir)
        print (self.depth_dir)
        print (self.timestamp_file)
        
        self.bridge = CvBridge()
        self.frame = frame

        self.name = name
        self.name_raw = name_raw
        self.camera= camera
        self.radius = 5
        self.bag_name = "kitti_{}_drive_{:04d}_{}.bag".format(date, drive, name)
        self.radius = radius
    
    def write(self):
        compression = rosbag.Compression.NONE
        self.bag = rosbag.Bag(self.bag_name, 'w', compression=compression)
        self.__loadDateTime()
        self.__mask_and_write()
        self.bag.close()



    def __loadDateTime(self):
        self.timestamps = []
        utc = pytz.utc
        _EPOCH = dt.datetime(1970, 1, 1, tzinfo=pytz.utc)
        with open (self.timestamp_file, 'r') as f:
            for line in f.readlines():
                #print (line[0:-11])
                #print (line[-10:])
                t = dt.datetime.strptime(line[:-4], '%Y-%m-%d %H:%M:%S.%f')
                #frac = int(line[-10:-4]) 
                #dec = int (t.timestamp())
                t_utc = utc.localize(t)
                t_utc_timestamp = (t_utc - _EPOCH).total_seconds()


                frac, dec = math.modf(t_utc_timestamp)
                dec = int(dec)
                frac = int (frac * 10e8)

                print ('{}:{}'.format(dec, frac))

                rosstamp = rospy.Time(dec, frac)
                #print ('{:020d}'.format(rosstamp))
                print (rosstamp)
                self.timestamps.append(rosstamp)

        print (len(self.timestamps))

    def __mask_and_write(self):
        for m in range(len(self.timestamps)):
            image_filename = '{:010d}.png'.format(m)
            raw_if = os.path.join(self.depth_raw_dir, image_filename)
            dense_if = os.path.join(self.depth_dir, image_filename)
            if not os.path.exists(raw_if) or not os.path.exists(dense_if):
                continue

            print (raw_if)
            print (dense_if)
            raw_img = cv2.imread(raw_if, cv2.IMREAD_ANYDEPTH)
            dense_img = cv2.imread(dense_if, cv2.IMREAD_ANYDEPTH)

            min_y = np.ones(shape=(raw_img.shape[1]), dtype=int) * raw_img.shape[0]

            for i in range(raw_img.shape[1]):
                #print('{} / {}'.format(i, raw_img.shape[1]))
                start_j = max(i-self.radius, 0)
                end_j = min(i+ self.radius, raw_img.shape[1])
                for j in range(start_j, end_j):
                    for k in range(raw_img.shape[0]):
                        if k > min_y[i]:
                            break
                        if raw_img[k,j] > 0:
                            min_y[i] = k
                            break
                #print (min_y[i])
            
            empty = True
            while empty:
                empty = False
                for i in range(1, min_y.shape[0]-1):
                    if min_y[i] == raw_img.shape[0]:
                        min_y[i] = min(min_y[i-1], min_y[i+1])
                        empty=True
                #print (empty)
            
            if min_y[0] == raw_img.shape[0]:
                min_y[0] = min_y[1]
            if min_y[-1] == raw_img.shape[0]:
                min_y[-1] = min_y[-2]

            for i in range(dense_img.shape[1]):
                for j in range(dense_img.shape[0]):
                    if j < min_y[i]:
                        dense_img[j,i] =0
      
            dense_rimg = self.bridge.cv2_to_imgmsg(dense_img, "mono16")
            dense_rimg.header.stamp = self.timestamps[m]
            dense_rimg.header.frame_id = self.frame
            self.bag.write(self.camera+'/'+self.name, dense_rimg, self.timestamps[m])
            raw_rimg = self.bridge.cv2_to_imgmsg(raw_img, "mono16")
            raw_rimg.header.stamp = self.timestamps[m]
            raw_rimg.header.frame_id = self.frame
            self.bag.write(self.camera+'/'+self.name_raw, raw_rimg, self.timestamps[m])




if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--date', type=str, default='2011_09_26', help='date of the data')
    parser.add_argument('--drive', type=int, default=93, help='drive of the data')
    parser.add_argument('--base', type=str, required=True, help='base dir path')
    parser.add_argument('--depth', type=str, required=True, help='depth dir path')
    parser.add_argument('--depth_raw', type=str, required=True, help='raw depth dir path')
    parser.add_argument('--camera', type=str, default='/kitti/camera_color_left')
    parser.add_argument('--name', type=str, default='depth')
    parser.add_argument('--name_raw', type=str, default='depth_raw')
    parser.add_argument('--frame', type=str, default='camera_color_left', help= 'frame id for depth')

    args = parser.parse_args()

    print(args)

    dw = DepthWriter(args.base, args.date, args.drive, args.depth, args.depth_raw, args.name, args.name_raw, args.camera, args.frame)
    dw.write()

