#!/bin/bash

# Change Directory to vizflyt_ws
conda activate vizflyt
colcon build --symlink-install
source install/setup.bash
source install/local_setup.bash

export PYTHON_EXECUTABLE="$HOME/miniconda3/envs/vizflyt/bin/python"
export PYTHONPATH="$HOME/miniconda3/envs/vizflyt/lib/python3.10/site-packages:$PYTHONPATH"
export PYTHONPATH=$PYTHONPATH:/home/pear_group/VizFlyt/nerfstudio

