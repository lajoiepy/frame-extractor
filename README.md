This package extracts keyframe images for which loop closures have been detected.

It was tested on ROS 2 Foxy, but should work on other distributions.

# Installation steps
- Clone the repository into a ROS 2 workspace
- `rosdep install --from-paths src --ignore-src --rosdistro foxy -y --skip-keys "gtsam"`
- Install GTSAM https://github.com/borglab/gtsam
- `colcon build --symlink-install`

# Run the SLAM frame extractor
- Modify the configuration file in `config/` according to your local setup.
- Copy/Modify the `launch/kitti_experiment.launch.py` to fit your system. In particular, specify the path to your path. Details about the parameters are available [here](config/slam_frame_extractor/README.md).
- Launch the experiment launch file.

# Going further
- You can use any odometry source, see `launch/kitti_experiment.launch.py`.
- There is currently only a `stereo_handler.cpp`, but you can easily implement a handler for different sensors.
- You can also easily use another Visual Place Recognition network than NetVLAD by implementing a new technique by respecting the template of `netvlad.py` and changing the import in `global_image_descriptor_loop_closure_detection.py`.

If you use the code in this repository please cite:
```
@article{lajoieCalibration2022,
	title = {Self-{Supervised} {Domain} {Calibration} and {Uncertainty} {Estimation} for {Place} {Recognition} via {Robust} {SLAM}},
	url = {http://arxiv.org/abs/2203.04446},
	doi = {10.48550/arXiv.2203.04446},
	year = {2022},
	author = {Lajoie, Pierre-Yves and Beltrame, Giovanni},
}
```