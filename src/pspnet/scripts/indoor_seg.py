#!/usr/bin/env python

"""
Indoor image segmentation
Date: 06/07/2020
Author: Thuong
Run the code first and then add rosparam later
"""

from __future__ import division
from __future__ import print_function

import sys
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

import numpy as np

from sensor_msgs.msg import PointCloud2
from utils import PointType, ColorPclGenerator
import message_filters
import time

from ptsemseg.models import get_model
from ptsemseg.utils import convert_state_dict

from skimage.transform import resize
import cv2

import torch

def color_map(N=256, normalized=False):
    """
    Return Color Map in PASCAL VOC format (rgb)
    Agrs:
        N (int): number of classes
        normalized (bool): whether colors are normalized (float 0-1)
    Return:
        (Nx3 numpy array): a color map
    """
    def bitget(byteval, idx):
        return ((byteval & (1 << idx)) != 0)

    dtype = 'float32' if normalized else 'uint8'
    cmap = np.zeros((N, 3), dtype=dtype)
    for i in range(N):
        r = g = b = 0
        c = i
        for j in range(8):
            r = r | (bitget(c, 0) << 7-j)
            g = g | (bitget(c, 1) << 7-j)
            b = b | (bitget(c, 2) << 7-j)
            c = c >> 3
        cmap[i] = np.array([r, g, b])
    cmap = cmap/255.0 if normalized else cmap
    return cmap

def decode_segmap(temp, n_classes, cmap):
    """
    Given an image of class predictions, produce an bgr8 image with class colors
    Args:
        temp (2d numpy int array): input image with semantic classes (as integer)
        n_classes (int): number of classes
        (Nx3 numpy array): input color map
    Returns:
        (numpy array bgr8): the decoded image with class colors
    """
    r = temp.copy()
    g = temp.copy()
    b = temp.copy()
    for l in range(0, n_classes):
        r[temp == l] = cmap[l,0]
        g[temp == l] = cmap[l,1]
        b[temp == l] = cmap[l,2]
    bgr = np.zeros((temp.shape[0], temp.shape[1], 3))
    bgr[:, :, 0] = b
    bgr[:, :, 1] = g
    bgr[:, :, 2] = r
    return bgr.astype(np.uint8)

class Semantic:
    """
    Class for ros node to take in a color image (bgr) and do semantic segmantation
    on it to produce an image with semantic class colors (chair, desk etc.)
    CNN: PSPNet (https://arxiv.org/abs/1612.01105) (with resnet50) pretrained on ADE20K
    """
    def __init__(self):
        """
        Constructor
        """
        # Get point type
        point_type = rospy.get_param('/semantic/point_type')
        if point_type == 0:
            self.point_type = PointType.COLOR
            print('Generate color point cloud.')
        elif point_type == 1:
            self.point_type = PointType.SEMANTICS_MAX
            print('Generate semantic point cloud [max fusion].')
        elif point_type == 2:
            self.point_type = PointType.SEMANTICS_BAYESIAN
            print('Generate semantic point cloud [bayesian fusion].')
        else:
            print("Invalid point type.")
            return
        # Get image size
        self.img_width, self.img_height = rospy.get_param('/semantic/width'), rospy.get_param('/semantic/height')
        # Set up CNN is use semantics
        if self.point_type is not PointType.COLOR:

            print('Setting up CNN model...')
            # Set device
            self.device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")
            print("Device: ", self.device)
            # Get dataset
            dataset = rospy.get_param('/semantic/dataset')
            # Setup model
            model_name ='pspnet'
            model_path = rospy.get_param('/semantic/model_path')

            if dataset == 'sunrgbd': # If use version fine tuned on sunrgbd dataset
                self.n_classes = 38 # Semantic class number
                self.model = get_model(model_name, self.n_classes, version = 'sunrgbd_res50')
                state = torch.load(model_path, map_location=self.device)
                self.model.load_state_dict(state)
                width = rospy.get_param('/semantic/width')
                height = rospy.get_param('/semantic/height')
                self.cnn_input_size = (width, height)
                self.mean = np.array([104.00699, 116.66877, 122.67892]) # Mean value of dataset

            elif dataset == 'ade20k':
                self.n_classes = 150 # Semantic class number
                self.model = get_model(model_name, self.n_classes, version = 'ade20k')
                state = torch.load(model_path, map_location=self.device)
                self.model.load_state_dict(convert_state_dict(state['model_state'])) # Remove 'module' from dictionary keys
                width = rospy.get_param('/semantic/width')
                height = rospy.get_param('/semantic/height')
                self.cnn_input_size = (width, height)
                self.mean = np.array([104.00699, 116.66877, 122.67892]) # Mean value of dataset
            self.model = self.model.to(self.device)
            self.model.eval()
            self.cmap = color_map(N = self.n_classes, normalized = False) # Color map for semantic classes

        # Declare array containers for bayesian
        # It will increase the accuracy but large computational time
        if self.point_type is PointType.SEMANTICS_BAYESIAN:
            self.semantic_colors = np.zeros((3, self.img_height, self.img_width, 3), dtype = np.uint8)
            self.confidences = np.zeros((3, self.img_height, self.img_width), dtype = np.float32)

        # Set up ROS
        print('Setting up ROS...')
        self.bridge = CvBridge()

        # Semantic image publisher
        self.sem_img_pub = rospy.Publisher("/semantic/semantic_image", Image, queue_size = 1)

        self.color_sub = message_filters.Subscriber(rospy.get_param('/semantic/color_image_topic'),
                                                    Image, queue_size = 1, buff_size = 30*480*640)
        self.depth_sub = message_filters.Subscriber(rospy.get_param('/semantic/depth_image_topic'),
                                                    Image, queue_size = 1, buff_size = 40*480*640 )
        self.ts = message_filters.ApproximateTimeSynchronizer([self.color_sub, self.depth_sub],
                                                    queue_size = 1, slop = 0.3)
        self.ts.registerCallback(self.color_depth_callback)

        print('Ready.')

    def color_depth_callback(self, color_img_ros, depth_img_ros):
        """
        Callback function to produce point cloud registered with semantic class
        color based on input color image and depth image
        Args:
            color_img_ros (sensor_msgs.Image): the input color image (bgr8)
            depth_img_ros (sensor_msgs.Image): the input depth image (registered
                                               to the color image frame) (float32)
                                               values are in meters
        Returns:
            semantic_color_image publish as ROS node
        """

        # Convert ros Image message to numpy array
        try:
            color_img = self.bridge.imgmsg_to_cv2(color_img_ros, "bgr8")
            depth_img = self.bridge.imgmsg_to_cv2(depth_img_ros, "32FC1")
        except CvBridgeError as e:
            print(e)
        # Resize depth
        if depth_img.shape[0] is not self.img_height or depth_img.shape[1] is not self.img_width:
            depth_img = resize(depth_img, (self.img_height, self.img_width),
                               order = 0, mode = 'reflect', anti_aliasing=False, preserve_range = True)
            depth_img = depth_img.astype(np.float32)

        # Do semantic segmantation
        if self.point_type is PointType.SEMANTICS_MAX:
            start = time.time()
            semantic_color, pred_confidence = self.predict_max(color_img)
            end = time.time()
            # Check processing time
            print("Processing time: ", end-start)


        elif self.point_type is PointType.SEMANTICS_BAYESIAN:
            self.predict_bayesian(color_img)

        # Publish semantic image
        if self.sem_img_pub.get_num_connections() > 0:
            if self.point_type is PointType.SEMANTICS_MAX:
                semantic_color_msg = self.bridge.cv2_to_imgmsg(semantic_color, encoding="bgr8")
            else:
                semantic_color_msg = self.bridge.cv2_to_imgmsg(self.semantic_colors[0], encoding="bgr8")
            self.sem_img_pub.publish(semantic_color_msg)

    def predict_max(self, img):
        """
        Do semantic prediction for max fusion
        Args:
            img (numpy array rgb8)
        Returns:
            semantic_color (numpy array rgb8): mask color image for segmentation
            pred_confidence (numpy array): prediction confidence
        """
        class_probs = self.predict(img)

        # Take best prediction and confidence
        pred_confidence, pred_label = class_probs.max(1)
        pred_confidence = pred_confidence.squeeze(0).cpu().numpy()
        pred_label = pred_label.squeeze(0).cpu().numpy()
        pred_label = resize(pred_label, (self.img_height, self.img_width),
                            order = 0, mode = 'reflect', anti_aliasing=False, preserve_range = True)
        pred_label = pred_label.astype(np.int)

        # Add semantic color
        semantic_color = decode_segmap(pred_label, self.n_classes, self.cmap)
        pred_confidence = resize(pred_confidence, (self.img_height, self.img_width),
                                 mode = 'reflect', anti_aliasing=True, preserve_range = True)
        return (semantic_color, pred_confidence)

    def predict(self, img):
        """
        Do semantic segmantation
        Args:
            img: (numpy array bgr8) The input cv image
        Returns:
            semantic_img: image with semantic label in pixel
        """
        img = img.copy()

        img = resize(img, self.cnn_input_size, mode = 'reflect',
                    anti_aliasing=True, preserve_range = True)
        img = img.astype(np.float32)
        img -= self.mean
        # Convert HWC -> CHW for torch
        img = img.transpose(2, 0, 1)
        # Convert to tensor
        img = torch.tensor(img, dtype = torch.float32)
        img = img.unsqueeze(0) # Add batch dimension required by CNN
        with torch.no_grad():
            img = img.to(self.device)
            # Do inference
            since = time.time()
            outputs = self.model(img) #N,C,W,H
            # Apply softmax to obtain normalized probabilities
            outputs = torch.nn.functional.softmax(outputs, 1)
            return outputs

def main(args):
    rospy.init_node('semantic_cloud', anonymous=True)
    seg_cnn = Semantic()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

if __name__ == '__main__':
    main(sys.argv)
