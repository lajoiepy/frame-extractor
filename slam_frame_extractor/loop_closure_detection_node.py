#!/usr/bin/env python3

# Loop Closure Detection service
# Abstraction to support multiple implementations of loop closure detection for benchmarking

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

from slam_frame_extractor.global_image_descriptor_loop_closure_detection import GlobalImageDescriptorLoopClosureDetection


class LoopClosureDetection(Node):
    """ Global image descriptor matching for loop closure detection """

    def __init__(self):
        """Initialization and parameter parsing"""
        super().__init__('loop_closure_detection')

        self.declare_parameters(
            namespace='',
            parameters=[('frontend.distance_threshold', None),
                        ('frontend.global_descriptor_technique', None),
                        ('frontend.pca_checkpoint', None), ('frontend.nn_checkpoint', None),
                        ('nb_best_matches', 20), ('frontend.image_crop_size', None),
                        ('frontend.intra_loop_min_inbetween_keyframes', 10),
                        ('frontend.enable_loop_closures', False)])
        self.params = {}
        self.params['frontend.distance_threshold'] = self.get_parameter(
            'frontend.distance_threshold').value
        self.params['frontend.intra_loop_min_inbetween_keyframes'] = self.get_parameter(
            'frontend.intra_loop_min_inbetween_keyframes').value
        self.params['nb_best_matches'] = self.get_parameter(
            'nb_best_matches').value
        self.params['frontend.global_descriptor_technique'] = self.get_parameter(
            'frontend.global_descriptor_technique').value
        self.params['frontend.nn_checkpoint'] = self.get_parameter(
            'frontend.nn_checkpoint').value
        self.params['frontend.enable_loop_closures'] = self.get_parameter(
            'frontend.enable_loop_closures').value
        self.params["frontend.image_crop_size"] = self.get_parameter(
            'frontend.image_crop_size').value

        self.glcd = GlobalImageDescriptorLoopClosureDetection(
            self.params, self)


if __name__ == '__main__':

    rclpy.init(args=None)
    lcd = LoopClosureDetection()
    lcd.get_logger().info('Initialization done.')
    rclpy.spin(lcd)
    rclpy.shutdown()
