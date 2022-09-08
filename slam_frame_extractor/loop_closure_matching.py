import numpy as np
from slam_frame_extractor.nn_matching import NearestNeighborsMatching

class LoopClosureMatching(object):
    """Sparse matching for loop closure detection
        Matches global descriptors to generate loop closure candidates
        Then candidates are selected such that we respect the communication budget
    """

    def __init__(self, params):
        """ Initialization of loop closure matching

        Args:
            params (dict): ROS 2 parameters
        """
        # Extract params
        self.params = params
        # Initialize matching structs
        self.local_nnsm = NearestNeighborsMatching()

    def add_local_global_descriptor(self, embedding, id):
        """ Add a local keyframe for matching

        Args:
            embedding (np.array): global descriptor
            id (int): keyframe id
        """
        self.local_nnsm.add_item(embedding, id)

    def match_local_loop_closures(self, descriptor, kf_id):
        kfs, ds = self.local_nnsm.search(descriptor,
                                         k=2*self.params['frontend.intra_loop_min_inbetween_keyframes'])

        if len(kfs) > 0 and kfs[0] == kf_id:
            kfs, ds = kfs[1:], ds[1:]
        if len(kfs) == 0:
            return None, None

        for kf, d in zip(kfs, ds):
            if abs(kf -
                   kf_id) < self.params['frontend.intra_loop_min_inbetween_keyframes']:
                continue

            if d > self.params['frontend.distance_threshold']:
                continue

            others = [i for i in kfs if abs(i - kf) > self.params['frontend.intra_loop_min_inbetween_keyframes'] and abs(i - kf_id) > self.params['frontend.intra_loop_min_inbetween_keyframes']]
            return kf, others
        return None, None