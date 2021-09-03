#!/usr/bin/env python
"""
@file dataCollection.py .

@author Ayush Saini
@email ayush@robrosystems.com
@brief Data Collection using ROS Topic
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


class DataCollection:
    """Data Collection using ROS Topic."""

    def __init__(self):
        """Initialize ROS node, get parameters , initialise cloud library."""
        rospy.init_node('dataCollection', anonymous=True)
        proj_name = rospy.get_param('~proj_name')
        image_topic_name = rospy.get_param('~image_topic_name')
        image_type = rospy.get_param('~image_type')
        self.camera_name = rospy.get_param('~camera_name')
        cred_path = rospy.get_param('~cred_path')
        self.grayscale = rospy.get_param('~grayscale')
        storage_class = rospy.get_param('~storage_class')
        storage_location = rospy.get_param('~storage_location')
        self.category = 'DEFAULT'
        try:
            self.category_param_valid = True
            self.category = rospy.get_param('~category')
        except:
            self.category_param_valid = False

        try:
            category_topic_valid = True
            category_topic_name = rospy.get_param('~category_topic_name')
        except:
            category_topic_valid = False

        if(category_topic_valid):
            rospy.Subscriber(
                category_topic_name, String, self.categoryRecievedCallback)

        try:
            self.resize_q = True
            self.image_size = rospy.get_param('~image_size')
        except:
            self.resize_q = False

        try:
            self.crop_q = True
            self.crop_size = rospy.get_param('~crop_size')
        except:
            self.crop_q = False

        os.environ['GOOGLE_APPLICATION_CREDENTIALS'] = cred_path

        client = storage.Client()
        proj_bucket = client.bucket(proj_name)
        if proj_bucket.exists():
            rospy.logdebug('bucket exists')
        else:
            rospy.logdebug('bucket does not exists, creating bucket..')
            proj_bucket.storage_class = storage_class
            try:
                client.create_bucket(proj_bucket, location=storage_location)
            except Exception as e:
                rospy.logerr(e)
        self.uploader_obj = RobroBucketImageUploader(
            proj_name, image_type, proj_name, cred_path)

        rospy.Subscriber(image_topic_name, Image, self.imageRecievedCallback)

    def categoryRecievedCallback(self, data):
        """Recieves Category of image.

        Args:
            data ([String])
        """
        rospy.logdebug('Recieved new Category')
        self.category = data.data

    def imageRecievedCallback(self, data):
        """Recieves ROS image converts to CV Image, apply transformations and upload.

        Args:
            data ([ROS Image])
        """
        rospy.logdebug('Recieved Image')
        try:
            cv_image = CvBridge().imgmsg_to_cv2(data, 'passthrough')
        except CvBridgeError as e:
            rospy.logerr(e)
        if self.crop_q:
            cv_image = cv_image[
                int(self.crop_size.split(',')[1]):
                int(self.crop_size.split(',')[1])+int(
                    self.crop_size.split(',')[3]),
                int(self.crop_size.split(',')[0]):int(
                    self.crop_size.split(',')[0])
                + int(self.crop_size.split(',')[2])]
        if self.resize_q:
            dim = (int(self.image_size.split(',')[0]), int(
                self.image_size.split(',')[1]))
            cv_image = cv2.resize(cv_image, dim, interpolation=cv2.INTER_AREA)
        if self.grayscale:
            cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        if self.category_param_valid or self.category_topic_valid:
            self.uploader_obj.addToUploadQueue(
                cv_image, self.camera_name, self.category)
        else:
            self.uploader_obj.addToUploadQueue(cv_image, self.camera_name)


if __name__ == '__main__':
    DataCollection()
    rospy.spin()

