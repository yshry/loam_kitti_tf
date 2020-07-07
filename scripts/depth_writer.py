#!/usr/bin/env python
import rosbag
import rospy
import os
import math


import datetime as dt
from pytz import timezone
import pytz
import argparse
from sensor_msgs.msg import Image
from std_msgs.msg import Header
import cv2
from cv_bridge import CvBridge, CvBridgeError

class DepthWriter:
    def __init__(self, base_path, date, drive, depth_path, camera, name, frame, image_path='image_02'):

        self.main_dir = os.path.join(base_path, '{}/{}_drive_{:04d}_sync'.format(date,date, drive))
        self.depth_dir = os.path.join(self.main_dir, depth_path)
        self.timestamp_file = os.path.join(self.main_dir, '{}/timestamps.txt'.format(image_path))
        
        print (self.main_dir)
        print (self.depth_dir)
        print (self.timestamp_file)
        
        self.bridge = CvBridge()
        self.frame = frame

        compression = rosbag.Compression.NONE
        self.name = name
        self.bag = rosbag.Bag("kitti_{}_drive_{:04d}_{}.bag".format(date, drive, name), 'w', compression=compression)
        self.camera= camera
    
    def write(self):
        self.__loadDateTime()
        self.__writeDepth()


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

    def __writeDepth(self):
       
        for i in range(len(self.timestamps)):
            image_filename = '{:010d}.png'.format(i)
            image_filename = os.path.join(self.depth_dir, image_filename)
            if not os.path.exists(image_filename):
                continue
            
            print (image_filename)
            cv_image = cv2.imread(image_filename, cv2.IMREAD_ANYDEPTH)
            ros_image = self.bridge.cv2_to_imgmsg(cv_image, "mono16")
            ros_image.header.stamp = self.timestamps[i]
            ros_image.header.frame_id = self.frame
            self.bag.write(self.camera+'/'+self.name, ros_image, self.timestamps[i])



if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--date', type=str, default='2011_09_26', help='date of the data')
    parser.add_argument('--drive', type=int, default=93, help='drive of the data')
    parser.add_argument('--base', type=str, default='/data/dataset/menandro', help='base dir path')
    parser.add_argument('--depth', type=str, default='output/upsampling', help='depth dir path')
    parser.add_argument('--camera', type=str, default='/kitti/camera_color_left')
    parser.add_argument('--name', type=str, default='depth')
    parser.add_argument('--frame', type=str, default='camera_color_left', help= 'frame id for depth')

    args = parser.parse_args()

    print(args)

    dw = DepthWriter(args.base, args.date, args.drive, args.depth, args.camera, args.name, args.frame)
    dw.write()

