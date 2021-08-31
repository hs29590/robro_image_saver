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

from std_msgs.msg import String

rospy.init_node('dataCollection', anonymous=True)

proj_name = rospy.get_param('~proj_name')
ros_topic_name = rospy.get_param('~ros_topic_name')
image_type = rospy.get_param('~image_type')
camera_name = rospy.get_param('~camera_name')
cred_path = rospy.get_param('~cred_path')
category = 'DEFAULT'
try:
    category_param_valid = True
    category = rospy.get_param('~category')
except:
    category_param_valid = False

try:
    category_topic_valid = True
    category_topic_name = rospy.get_param('~category_topic_name')
except:
    category_topic_valid = False

try:
    resize_q = True
    image_size = rospy.get_param('~image_size')
except:
    resize_q = False

try:
    crop_q = True
    crop_size = rospy.get_param('~crop_size')
except:
    crop_q = False

grayscale = rospy.get_param('~grayscale')


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
uploader_obj = RobroBucketImageUploader(
    proj_name, image_type, proj_name, cred_path)


def categoryRecievedCallback(data):
    """Recieves Category of image.

    Args:
        data ([String])
    """
    print('Recieved new Category')
    global category
    category = data.data


def imageRecievedCallback(data):
    """Recieves ROS image converts to CV Image, apply transformations and upload.

    Args:
        data ([ROS Image])
    """
    print('Recieved Image')
    try:
        cv_image = CvBridge().imgmsg_to_cv2(data, 'passthrough')
    except CvBridgeError as e:
        print(e)
    if crop_q:
        cv_image = cv_image[
            int(crop_size.split(',')[1]):
            int(crop_size.split(',')[1])+int(crop_size.split(',')[3]),
            int(crop_size.split(',')[0]):int(crop_size.split(',')[0])
            + int(crop_size.split(',')[2])]
    if resize_q:
        dim = (int(image_size.split(',')[0]), int(image_size.split(',')[1]))
        cv_image = cv2.resize(cv_image, dim, interpolation=cv2.INTER_AREA)
    if grayscale:
        cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
    if category_param_valid or category_topic_valid:
        uploader_obj.addToUploadQueue(cv_image, camera_name, category)
    else:
        uploader_obj.addToUploadQueue(cv_image, camera_name)


image_sub = rospy.Subscriber(ros_topic_name, Image, imageRecievedCallback)
if category_topic_valid:
    category_sub = rospy.Subscriber(
        category_topic_name, String, categoryRecievedCallback)

rospy.spin()
