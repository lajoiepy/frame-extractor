#!/usr/bin/env python
import numpy as np
from cv_bridge import CvBridge

import os
from os.path import join, exists, isfile, realpath, dirname
import numpy as np

from slam_frame_extractor.netvlad import NetVLAD

from slam_frame_extractor.loop_closure_matching import LoopClosureMatching

from slam_interfaces.msg import (KeyframeRGB, LocalKeyframeMatches)

import rclpy
from rclpy.node import Node


class GlobalImageDescriptorLoopClosureDetection(object):
    """ Global Image descriptor matching """

    def __init__(self, params, node):
        """Initialization

        Args:
            params (dict): parameters
            node (ROS 2 node handle): node handle
        """
        self.params = params
        self.node = node

        self.lcm = LoopClosureMatching(params)

        # Place Recognition network setup
        if self.params['frontend.global_descriptor_technique'].lower(
        ) == 'netvlad':
            self.params['frontend.pca_checkpoint'] = self.node.get_parameter(
                'frontend.pca_checkpoint').value
            self.global_descriptor = NetVLAD(self.params, self.node)
        else:
            self.node.get_logger().err(
                'ERROR: Unknown technique. Using NetVLAD as default.')
            self.params['frontend.pca_checkpoint'] = self.node.get_parameter(
                'frontend.pca_checkpoint').value
            self.global_descriptor = NetVLAD(self.params, self.node)

        # ROS 2 objects setup
        self.receive_keyframe_subscriber = self.node.create_subscription(
            KeyframeRGB, 'keyframe_data', self.receive_keyframe, 100)

        self.local_match_publisher = self.node.create_publisher(
            LocalKeyframeMatches, 'local_keyframe_match', 100)

    def add_global_descriptor_to_map(self, embedding, kf_id):
        """ Add global descriptor to matching list

        Args:
            embedding (np.array): descriptor
            kf_id (int): keyframe ID
        """
        # Add for matching
        self.lcm.add_local_global_descriptor(embedding, kf_id)
        # Local matching
        self.detect_intra(embedding, kf_id)

    def detect_intra(self, embedding, kf_id):
        """ Detect loop closures

        Args:
            embedding (np.array): descriptor
            kf_id (int): keyframe ID

        Returns:
            list(int): matched keyframes
        """
        if self.params['frontend.enable_loop_closures']:
            kf_match, kf_best_matches = self.lcm.match_local_loop_closures(
                embedding, kf_id)
            if kf_match is not None:
                msg = LocalKeyframeMatches()
                msg.keyframe0_id = kf_id
                msg.keyframe1_id = kf_match
                msg.other_matches_keyframe_ids = kf_best_matches
                self.local_match_publisher.publish(msg)

    def receive_keyframe(self, msg):
        """Callback to add a keyframe 

        Args:
            msg (slam_interfaces::msg::KeyframeRGB): Keyframe data
        """
        # Netvlad processing
        bridge = CvBridge()
        cv_image = bridge.imgmsg_to_cv2(msg.image,
                                        desired_encoding='passthrough')
        embedding = self.global_descriptor.compute_embedding(cv_image)

        self.add_global_descriptor_to_map(embedding, msg.id)