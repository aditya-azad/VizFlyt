#!/bin/bash

eval "$(conda shell.bash hook)"
conda activate vizflyt

# check packages
if [ ! -d "./src" ]; then
  echo "⚠️  Packages not found. Run 'clone.sh' script to clone the packages before building."
fi

# check if previously built or not
if [ -f "./install/setup.bash" ]; then
  echo "📦 Sourcing workspace from ./install/setup.bash"
  source ./install/setup.bash
elif [ -f "./install/local_setup.bash" ]; then
  echo "📦 Sourcing workspace from ./install/local_setup.bash"
  source ./install/local_setup.bash
elif [ -f "/opt/ros/humble/setup.bash" ]; then
  echo "📦 Sourcing ROS Humble"
  source /opt/ros/humble/setup.bash
else
  echo "⚠️  ROS installation not found"
fi

export ROS_DISTRO=humble
export PYTHON_EXECUTABLE="$HOME/miniconda3/envs/vizflyt/bin/python"
export PYTHONPATH="$HOME/miniconda3/envs/vizflyt/lib/python3.10/site-packages:$PYTHONPATH"
export PYTHONPATH=$PYTHONPATH:$HOME/VizFlyt/nerfstudio
