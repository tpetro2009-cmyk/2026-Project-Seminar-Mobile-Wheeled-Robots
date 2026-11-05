#!/usr/bin/env bash
for n in 0 0.02 0.1; do
  ros2 launch odometry_study experiment.launch.py noise:=$n results_dir:=$PWD/results
done
