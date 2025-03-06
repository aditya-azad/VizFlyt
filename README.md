# VizFlyt: A Perception-centric Pedagogical Framework For Autonomous Aerial Robots. 

<span style="color:red;">⚠️ This repo is still under construction and not a finished work. It will be updated incrementally over the next few days until finished.</span>


## Installation Instructions

- ROS2 Installation

  Follow the Official ROS2 Humble Docs. 

- Create Conda Environment

  ```bash
  conda create --name vizflyt -y python=3.10.14
  
  conda activate vizflyt
  
  python -m pip install --upgrade pip

  pip uninstall torch torchvision functorch tinycudann
  ```

- Install PyTorch

  ```bash
  pip install torch==2.1.2+cu118 torchvision==0.16.2+cu118 --extra-index-url https://download.pytorch.org/whl/cu118
  
  conda install -c "nvidia/label/cuda-11.8.0" cuda-toolkit
  
  pip install ninja git+https://github.com/NVlabs/tiny-cuda-nn/#subdirectory=bindings/torch
  ```

- Clone the VizFlyt Repository along with modified nerfstudio. 

  ```bash
  git clone https://github.com/pearwpi/VizFlyt.git

  cd nerfstudio

  pip install --upgrade pip setuptools

  pip install -e .
  ```

- Setup vizflyt ros2 workspace

  ```bash
  pip install --upgrade "numpy<2"

  pip install transforms3d gdown
  ```

- Download data for running pretrained models

  ```bash
  cd vizflyt_ws/src/
  
  chmod +x download_data_and_outputs.sh
  
  ./download_data_and_outputs.sh
  ```

- Preparing your Environment {Digital Twin Generation using Nerfstudio}

  ```bash
  cd vizflyt_ws/src/vizflyt_viewer/

  # Training your Environment 
  python scripts/train.py splatfacto --data ./data/washburn-env6-itr0-1fps_nf_format/ --output-dir outputs/washburn-env6-itr0-1fps
  
  # Viewing your Environment
  python scripts/run_viewer.py --load-config ./outputs/washburn-env6-itr0-1fps/washburn-env6-itr0-1fps_nf_format/splatfacto/2025-03-06_032319/config.yml
  ```

- Setting Initial Pose and Camera Settings and Render Settings using GUI


  - Open the Viewer using, 

  ```bash
  cd vizflyt_ws/src/vizflyt_viewer/

  python scripts/run_viewer.py --load-config ./outputs/washburn-env6-itr0-1fps/washburn-env6-itr0-1fps_nf_format/splatfacto/2025-03-06_032319/config.yml
  ```
  
  - and set the following: 

    - Set Initial Position and Orientation that you want your vehicle to start from using GUI.  
    - Go to *Control Tab*, adjust the "Max Res" slider to set the rendering resolution you want at the output. 
    - Go to *Render Tab*, adjust the "Default FOV" slider to set the FOV you want at the output.

  - After everything is set, press the "Save Camera Pose" button to save your configuration. 


## Running the HITL Framework using ROS2
