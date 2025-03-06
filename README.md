# VizFlyt
## Installation Instructions

- Create Conda Environment

  ```bash
  conda create --name vizflyt -y python=3.10.14
  
  conda activate vizflyt
  
  python -m pip install --upgrade pip
  ```

- Install PyTorch

  ```bash
  pip install torch==2.1.2+cu118 torchvision==0.16.2+cu118 --extra-index-url https://download.pytorch.org/whl/cu118
  
  conda install -c "nvidia/label/cuda-11.8.0" cuda-toolkit
  
  pip install ninja git+https://github.com/NVlabs/tiny-cuda-nn/#subdirectory=bindings/torch
  ```

- Clone the VizFlyt Repository along with modified nerfstudio. 

  ```bash
  ```

- 
