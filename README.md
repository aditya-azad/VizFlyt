<span style="color:red;">⚠️ This repo is still under construction and not a finished work. It will be updated incrementally over the next few days until finished.</span>


<p align="center">
    <!-- project badges -->
    <a href="https://pear.wpi.edu/research/vizflyt.html"><img src="https://img.shields.io/badge/Project-Page-ffa"/></a>
    <!-- paper badges -->
    <a href="ARXIV_PAPER_URL">
        <img src='https://img.shields.io/badge/arXiv-Page-aff'>
    </a>
</p>


<div align="center">
<!-- <h1 style="font-size:2.0em;">VizFlyt</h3> -->
<picture>
    <source media="(prefers-color-scheme: dark)" srcset="assets/photos/vizflyt_logo.png" />
    <img alt="YOUR_PROJECT_NAME logo" src="assets/photos/vizflyt_logo.png" width="80%"/>
</picture>
<picture>
    <source media="(prefers-color-scheme: dark)" srcset="assets/photos/banner_image_vizflyt.png" />
    <img alt="YOUR_PROJECT_NAME logo" src="assets/photos/vizflyt_logo.png" width="80%"/>
</picture>
</div>


<div align="center">
<h3 style="font-size:2.0em;">A Perception Centric Pedagogical Framework for Autonomous Aerial Robots</h3>
<h4>Accepted for ICRA 2025</h4>
</div>
<div align="center">

[Quickstart](#quickstart) ·
[Planned Features](#planned-featurestodos) ·
[Project page](https://pear.wpi.edu/research/vizflyt.html)

</div>

# About

<!-- Autonomous aerial robots are rapidly becoming integral to various industries, necessitating hands-on aerial robotics courses to equip the next-generation workforce with practical expertise. A robust and efficient course framework requires a reliable testbed that ensures realistic testing without hardware risks.

VizFlyt is an open-source perception-centric Hardware-In-The-Loop (HITL) photorealistic testing framework designed to address this need. By leveraging pose from an external localization system, VizFlyt generates real-time, photorealistic visual sensor data using 3D Gaussian Splatting, enabling stress-free validation of autonomy algorithms on aerial robots without the risk of crashes. The system achieves an update rate of over 100Hz, ensuring high-fidelity real-time performance.

Beyond just a framework, VizFlyt introduces a new open-source and open-hardware curriculum, shaping the future of hands-on aerial robotics education. We have validated VizFlyt across multiple real-world HITL experiments, demonstrating its effectiveness and vast potential for research and education.

💡 Want to contribute? Whether you're developing new autonomy algorithms, integrating additional sensors, or creating novel datasets, VizFlyt is designed as a community-driven platform for advancing perception-driven aerial autonomy. -->
Testing autonomy algorithms for aerial robots in real-world environments is challenging due to safety risks and hardware limitations. VizFlyt is an open-source perception-centric Hardware-In-The-Loop (HITL) testing framework designed to enable photorealistic, real-time evaluation of autonomy stacks without the risk of crashes.

By leveraging pose from an external localization system, VizFlyt hallucinates onboard visual sensors using 3D Gaussian Splatting, achieving a 100Hz+ update rate for high-fidelity perception and control. This allows for safe and scalable Sim2Real validation, bridging the gap between simulation and real-world deployment. VizFlyt also supports an open-source and open-hardware curriculum for hands-on aerial robotics education.

💡 Want to contribute? Whether it’s new autonomy algorithms, sensor integrations, or datasets, VizFlyt is a community-driven platform advancing aerial robotics research.

# Quickstart

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
  cd vizflyt_ws/src/

  # Training your Environment 
  python vizflyt_viewer/scripts/train.py splatfacto --data ./vizflyt_viewer/data/washburn-env6-itr0-1fps_nf_format/ --output-dir vizflyt_viewer/outputs/washburn-env6-itr0-1fps
  
  # Viewing your Environment
  python vizflyt_viewer/scripts/run_viewer.py --load-config ./vizflyt_viewer/outputs/washburn-env6-itr0-1fps/washburn-env6-itr0-1fps_nf_format/splatfacto/2025-03-06_201843/config.yml

  # Exporting the Occupancy Grid Map for Collision Detection.
  python vizflyt_viewer/scripts/exporter.py gaussian-splat --load-config ./vizflyt_viewer/outputs/washburn-env6-itr0-1fps/washburn-env6-itr0-1fps_nf_format/splatfacto/2025-03-06_201843/config.yml --output-dir ./vizflyt_viewer/occupancy_grid/ 
  ```

- Setting Initial Pose and Camera Settings and Render Settings using GUI

  - Open the Viewer using, 

  ```bash
  cd vizflyt_ws/src/

  python vizflyt_viewer/scripts/run_viewer.py --load-config ./vizflyt_viewer/outputs/washburn-env6-itr0-1fps/washburn-env6-itr0-1fps_nf_format/splatfacto/2025-03-06_201843/config.yml
  ```
  
  - and set the following: 

    - Set Initial Position and Orientation that you want your vehicle to start from using GUI.  
    - Go to *Control Tab*, adjust the "Max Res" slider to set the rendering resolution you want at the output. 
    - Go to *Render Tab*, adjust the "Default FOV" slider to set the FOV you want at the output.

  - After everything is set, press the "Save Camera Pose" button to save your configuration. 


## Running the HITL Framework using ROS2
