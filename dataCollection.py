#!/usr/bin/env python
"""
@file dataCollection.py .

@author Ayush Saini
@email ayush@robrosystems.com
@brief script for Data Collection using ROS Topic
@version 0.1.0
@date 2021-08-28
@copyright Copyright (c) 2021
"""
import os

import cv2

from cv_bridge import CvBridge, CvBridgeError

from google.cloud import storage

from robro_cloud.robroBucketImageUploader import RobroBucketImageUploader

import rospy

from sensor_msgs.msg import Image

rospy.init_node('datacollector', anonymous=True)
print('** Enter every value as string **')
proj_name = str(input('Enter Project Name: '))
ros_topic_name = (input('Enter ROS topic name of camera: '))
image_type = (input('Enter Image Type: '))
camera_name = (input('Camera Name: '))
cred_path = (input('Input path to service account credentials json: '))
category_valid = (input('Upload data in different categories? Y or N : '))
if category_valid == 'Y':
    category = str(input('Input Category Name: '))

resize_q = input('Resize? Y or N ')
if resize_q == 'Y':
    image_size = input('Enter target size(Width,Height): ')
crop_q = input('Crop? Y or N ')
if crop_q == 'Y':
    crop_size = input('Enter target size(x,y,width,height): ')
grayscale = input('Grayscale? Y or N ')

os.environ['GOOGLE_APPLICATION_CREDENTIALS'] = cred_path

client = storage.Client()
proj_bucket = client.bucket(proj_name)
if proj_bucket.exists():
    print('bucket exists')
else:
    print('bucket does not exists, creating bucket..')
    proj_bucket.storage_class = 'STANDARD'
    try:
        client.create_bucket(proj_bucket, location='us-central1')
    except Exception as e:
        print(e)
if category_valid == 'Y':
    uploader_obj = RobroBucketImageUploader(
        proj_name, image_type, proj_name, camera_name, cred_path,category)
else:
    uploader_obj = RobroBucketImageUploader(
        proj_name, image_type, proj_name, camera_name, cred_path)


def imageRecievedCallback(data):
    """Recieves ROS image converts to CV Image, apply transformations and upload.

    Args:
        data ([ROS Image])
    """
    try:
        cv_image = CvBridge().imgmsg_to_cv2(data, 'passthrough')
    except CvBridgeError as e:
        print(e)
    if resize_q == 'Y':
        dim = (int(image_size.split(',')[0]), int(image_size.split(',')[1]))
        cv_image = cv2.resize(cv_image, dim, interpolation=cv2.INTER_AREA)
    if crop_q == 'Y':
        cv_image = cv_image[
            int(crop_size.split(',')[1]):
            int(crop_size.split(',')[1])+int(crop_size.split(',')[3]),
            int(crop_size.split(',')[0]):int(crop_size.split(',')[0])
            + int(crop_size.split(',')[2])]
    if grayscale == 'Y':
        cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
    uploader_obj.addToUploadQueue(cv_image)


image_sub = rospy.Subscriber(ros_topic_name, Image, imageRecievedCallback)

rospy.spin()
