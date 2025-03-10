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

[Quickstart](#quickstart-guide) · [Project Page](https://pear.wpi.edu/research/vizflyt.html)
  

</div>

---
# **About**

<!-- Autonomous aerial robots are rapidly becoming integral to various industries, necessitating hands-on aerial robotics courses to equip the next-generation workforce with practical expertise. A robust and efficient course framework requires a reliable testbed that ensures realistic testing without hardware risks.

VizFlyt is an open-source perception-centric Hardware-In-The-Loop (HITL) photorealistic testing framework designed to address this need. By leveraging pose from an external localization system, VizFlyt generates real-time, photorealistic visual sensor data using 3D Gaussian Splatting, enabling stress-free validation of autonomy algorithms on aerial robots without the risk of crashes. The system achieves an update rate of over 100Hz, ensuring high-fidelity real-time performance.

Beyond just a framework, VizFlyt introduces a new open-source and open-hardware curriculum, shaping the future of hands-on aerial robotics education. We have validated VizFlyt across multiple real-world HITL experiments, demonstrating its effectiveness and vast potential for research and education.

💡 Want to contribute? Whether you're developing new autonomy algorithms, integrating additional sensors, or creating novel datasets, VizFlyt is designed as a community-driven platform for advancing perception-driven aerial autonomy. -->
Testing autonomy algorithms for aerial robots in real-world environments is challenging due to safety risks and hardware limitations. VizFlyt is an open-source perception-centric Hardware-In-The-Loop (HITL) testing framework designed to enable photorealistic, real-time evaluation of autonomy stacks without the risk of crashes.

By leveraging pose from an external localization system, VizFlyt hallucinates onboard visual sensors using 3D Gaussian Splatting, achieving a 100Hz+ update rate for high-fidelity perception and control. This allows for safe and scalable Sim2Real validation, bridging the gap between simulation and real-world deployment. VizFlyt also supports an open-source and open-hardware curriculum for hands-on aerial robotics education.

💡 Want to contribute? Whether it’s new autonomy algorithms, sensor integrations, or datasets, VizFlyt is a community-driven platform advancing aerial robotics research.

---

# **Quickstart Guide**

This guide will walk you through setting up **VizFlyt**, installing dependencies, configuring the environment, and downloading necessary data.



## **1. Installation Instructions**

### **1.1 Prerequisites**
Ensure you have the following dependencies installed before proceeding:

- ✅ **[Ubuntu 22.04](https://releases.ubuntu.com/jammy/)**
- ✅ **[NVIDIA Drivers](https://www.nvidia.com/en-us/drivers/)** (For GPU acceleration)
- ✅ **[ROS2 Humble](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)** (Required for ROS-based workflows)
- ✅ **[Miniconda3](https://www.anaconda.com/docs/getting-started/miniconda/install#macos-linux-installation)** (For managing Python environments)
- ✅ **[Blender](https://www.blender.org/download/)** _(Optional: Use if testing without hardware)_

---

### **1.2 Setting Up the VizFlyt Environment**
#### **Step 1: Create and Activate the Conda Environment**
Run the following commands to set up a dedicated Conda environment for VizFlyt:

```bash
# Create a Conda environment with Python 3.10
conda create --name vizflyt -y python=3.10.14

# Activate the environment
conda activate vizflyt

# Upgrade pip
python -m pip install --upgrade pip
```

---

#### **Step 2: Install PyTorch with CUDA Support**
Install **PyTorch** and CUDA dependencies for GPU acceleration:

```bash
pip install torch==2.1.2+cu118 torchvision==0.16.2+cu118 --extra-index-url https://download.pytorch.org/whl/cu118

# Install CUDA Toolkit (Ensure compatibility with PyTorch version)
conda install -c "nvidia/label/cuda-11.8.0" cuda-toolkit

# Install tiny-cuda-nn (for optimized CUDA operations)
pip install ninja git+https://github.com/NVlabs/tiny-cuda-nn/#subdirectory=bindings/torch
```

---

#### **Step 3: Clone the VizFlyt Repository & Install Modified Nerfstudio**
Clone the VizFlyt repository and install the modified **Nerfstudio** framework:

```bash
# Clone the repository
git clone https://github.com/pearwpi/VizFlyt.git

cd VizFlyt/nerfstudio

# Upgrade pip and setuptools before installing dependencies
pip install --upgrade pip setuptools

# Install Nerfstudio in editable mode
pip install -e .
```

---

## **2. Making Your Workflow Easier**
To simplify your workflow, you can define **aliases** in your `~/.bashrc` file for frequently used commands. These are **optional but recommended**.

📌 Note: This workflow assumes that you have cloned VizFlyt in your home directory ($HOME/VizFlyt/). If your working directory is different, update the paths accordingly in the alias definitions.

#### **Step 1: Add Useful Aliases**
Append the following lines to your `~/.bashrc` or `~/.bash_profile`:

```bash
alias viz='conda activate vizflyt'
alias viz_ws='cd $HOME/VizFlyt/vizflyt_ws' 
alias source_ws='source install/setup.bash'
alias source_ws2='source install/local_setup.bash'
alias build_ws='colcon build --symlink-install'
alias set_env='export PYTHON_EXECUTABLE="$HOME/miniconda3/envs/vizflyt/bin/python" && export PYTHONPATH="$HOME/miniconda3/envs/vizflyt/lib/python3.10/site-packages:$PYTHONPATH" && export PYTHONPATH=$PYTHONPATH:$HOME/VizFlyt/nerfstudio'
alias init_vizflyt='viz && viz_ws && source_ws && source_ws2 && set_env && cd src'
```

#### **Step 2: Apply the Changes**
To make the aliases available immediately, run:

```bash
source ~/.bashrc
```

#### **Step 3: Initialize VizFlyt in Every New Terminal**
Now, every time you open a new terminal, simply run:

```bash
init_vizflyt
```

This command will:
✔️ Activate the **vizflyt** Conda environment  
✔️ Navigate to the **VizFlyt workspace**  
✔️ Source the required ROS2 setup files  
✔️ Set up the necessary **Python environment variables**  

---

## **3. Building the VizFlyt ROS2 Workspace**
Once your environment is set up, build the ROS2 workspace:

```bash
pip install --upgrade "numpy<2"
pip install transforms3d gdown

cd vizflyt_ws/

build_ws
```

This ensures all necessary dependencies are installed and the workspace is properly compiled.

---

## **4. Downloading Data & Pretrained Models**
To fetch required datasets and pre-trained models, run:

```bash
init_vizflyt  # Ensure the environment is set up

chmod +x download_data_and_outputs.sh  # Make script executable

./download_data_and_outputs.sh  # Run the script to download required data
```
---

### *


# **Generating a Digital Twin from Your Own Data Using Nerfstudio**

This guide provides step-by-step instructions for generating a high-fidelity **digital twin** using **Nerfstudio**. The workflow covers dataset preprocessing, training, visualization, and exporting an occupancy grid for collision detection.

## **1. Initialize the Workspace**
Ensure your workspace is set up correctly before proceeding:

```bash
init_vizflyt
```

---

## **2. Create a Nerfstudio-Compatible Dataset (SFM Step)**
If your input consists of images, convert them into a format suitable for **Nerfstudio**:

```bash
ns-process-data images \
  --data ./vizflyt_viewer/data/washburn-env6-itr0-1fps/ \
  --output-dir ./vizflyt_viewer/data/washburn-env6-itr0-1fps_nf_format/
```

---

## **3. Train the Environment Model**
Run the training process using **Splatfacto**, which will generate a **Gaussian Splatting-based** representation of the scene:

```bash
ns-train splatfacto \
  --data ./vizflyt_viewer/data/washburn-env6-itr0-1fps_nf_format/ \
  --output-dir vizflyt_viewer/outputs/washburn-env6-itr0-1fps
```

---

## **4. View the Trained Environment**
Visualize the generated digital twin using the **Nerfstudio Viewer**:

```bash
ns-viewer --load-config \
  ./vizflyt_viewer/outputs/washburn-env6-itr0-1fps/washburn-env6-itr0-1fps_nf_format/splatfacto/2025-03-06_201843/config.yml
```

## **5. Export the Occupancy Grid Map for Collision Detection**
Generate an **occupancy grid map** from the trained digital twin to use for **collision detection** in autonomous navigation:

```bash
ns-export gaussian-splat \
  --load-config ./vizflyt_viewer/outputs/washburn-env6-itr0-1fps/washburn-env6-itr0-1fps_nf_format/splatfacto/2025-03-06_201843/config.yml \
  --output-dir ./vizflyt_viewer/occupancy_grid/
```

---

## **6. Alternative: Run Python Scripts Directly**
If the global `ns-*` commands fail for any reason, you can manually execute the equivalent Python scripts with the same arguments:

- **Preprocess Data:**  
  ```bash
  python vizflyt_viewer/scripts/process_data.py
  ```
- **Train the Model:**  
  ```bash
  python vizflyt_viewer/scripts/train.py
  ```
- **View the Digital Twin:**  
  ```bash
  python vizflyt_viewer/scripts/run_viewer.py
  ```
- **Export the Occupancy Grid:**  
  ```bash
  python vizflyt_viewer/scripts/exporter.py
  ```

## **7. Configure Initial Pose, Camera Settings, and Render Settings**
To fine-tune the **initial pose**, **field of view (FOV)**, and **render resolution**, follow these steps:

### **Open the Viewer**
```bash
init_vizflyt

ns-viewer --load-config \
  ./vizflyt_viewer/outputs/washburn-env6-itr0-1fps/washburn-env6-itr0-1fps_nf_format/splatfacto/2025-03-06_201843/config.yml
```

![Set Render Settings](assets/photos/vizflyt-set-init-pose-demo.png)

### **Adjust the Following Settings in the GUI:**
1. **Set Initial Position & Orientation**  
   - Use the GUI to position the vehicle where you want it to start.

2. **Adjust Render Resolution**  
   - Navigate to the **Control Tab** and adjust the `"Max Res"` slider.

3. **Set Field of View (FOV)**  
   - Navigate to the **Render Tab** and adjust the `"Default FOV"` slider.

4. **Save Configuration**  
   - Once satisfied, click **"Save Render Settings"** to save your settings.


### **For More Control & Flexibility**
For advanced usage and fine-grained control over input/output parameters, refer to the official [Splatfacto documentation](https://docs.nerf.studio/nerfology/methods/splat.html#installation).

---


# Planned Features/TODOs

- [ ] Integrating with Blender Simulator
- [ ] Add Hardware Documentation
- [ ] Release code
- [ ] Adding multiple sensors (stereo, LiDAR, event cameras, etc.)
- [ ] Supporting dynamic scenes 

---

# Built On

<a href="https://github.com/nerfstudio-project/nerfstudio">
<picture>
    <source media="(prefers-color-scheme: dark)" srcset="https://raw.githubusercontent.com/nerfstudio-project/nerfstudio/main/docs/_static/imgs/logo-dark.png" />
    <img alt="tyro logo" src="https://raw.githubusercontent.com/nerfstudio-project/nerfstudio/main/docs/_static/imgs/logo.png" width="150px" />
</picture>
</a>

---

# Citation

If you use this code or find our research useful, please consider citing:

📌 Note: The VizFlyt framework is based on research accepted for publication at ICRA 2025. The final citation details will be updated once the paper is officially published.

```bibtex
@inproceedings{vizflyt2025,
  author    = {Kushagra Srivastava*, Rutwik Kulkarni*, Manoj Velmurugan*, Nitin J. Sanket},
  title     = {VizFlyt: An Open-Source Perception-Centric Hardware-In-The-Loop Framework for Aerial Robotics},
  booktitle = {IEEE International Conference on Robotics and Automation (ICRA)},
  year      = {2025},
  note      = {Accepted for publication},
  url       = {https://github.com/pearwpi/VizFlyt}
}
```

# Contributors

<a href="https://github.com/rkulkarni1999">
    <img src="https://avatars.githubusercontent.com/u/74806736?v=4" width="60px;" style="border-radius: 50%;"/>
</a>
<a href="https://github.com/Kush0301">
    <img src="https://avatars.githubusercontent.com/u/50519473?v=4" width="60px;" style="border-radius: 50%;"/>
</a>
<a href="https://github.com/vmanoj1996">
    <img src="https://avatars.githubusercontent.com/u/8917581?v=4" width="60px;" style="border-radius: 50%;"/>
</a>
<a href="https://github.com/NitinJSanket">
    <img src="https://avatars.githubusercontent.com/u/12091857?v=4" width="60px;" style="border-radius: 50%;"/>
</a>


<!-- \+ [nerfstudio contributors](https://github.com/nerfstudio-project/nerfstudio/graphs/contributors) -->


























