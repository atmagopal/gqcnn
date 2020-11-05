## Install Python virtual env

    sudo apt-get install python-pip
    sudo pip install virtualenv
    mkdir ~/.virtualenvs
    sudo pip install virtualenvwrapper
    export WORKON_HOME=~/.virtualenvs
    echo '. /usr/local/bin/virtualenvwrapper.sh' >> ~/.bashrc
    mkvirtualenv --python=python3 gqcnn

## Dependency: cv_bridge(OpenCV3) with Python3(.6) https://cyaninfinite.com/ros-cv-bridge-with-python-3/

    # Build a separate workspace for cv_bridge with Python3 to not 
    # conflict with Python2 default
    bash
    pip3 install rospkg catkin_pkg
    mkdir ~/cvbridge_build_ws/src
    cd ~/cvbridge_build_ws/src

    git clone -b melodic https://github.com/ros-perception/vision_opencv.git
    cd ..
    catkin config --init -DPYTHON_EXECUTABLE=/usr/bin/python3 -DPYTHON_INCLUDE_DIR=/usr/include/python3.6m -DPYTHON_LIBRARY=/usr/lib/x86_64-linux-gnu/libpython3.6m.so

        # If gqcnn is downloaded into another workspace, do:
        catkin config --extend <path to workspace with GQCNN ROS package>/devel
    
    catkin build

    # source this package when running gqcnn in virtual environment

    # Optional: alias to activate virtual environment and source this workspace in bash
    echo "alias gqcnn=workon gqcnn && source $HOME/cvbridge_build_ws/devel/setup.bash" > $HOME/.bash_aliases

## Dependencies in virtual environment

    bash
    workon gqcnn 
    
    # Install tensorflow https://www.tensorflow.org/install/pip
    # with GPU https://www.tensorflow.org/install/gpu
    # Ensure compatible NVIDIA, CUDA, cuDNN versions
    # Compatible with (older) tensorflow1
    pip install tensorflow-gpu==1.15 

    pip install "rtree>=0.8,<0.9" GPUtil psutil visualization
    
    cd ~/.virtualenvs/gqcnn/lib/python3.6/site-packages
    git clone https://github.com/BerkeleyAutomation/perception.git
    cd perception
    pip install -e .
    python setup.py test #test installation

## Get gqcnn
    cd ~/cvbridge_build_ws/src
    git clone https://github.com/BerkeleyAutomation/autolab_core.git
    git clone -b devel_hrg https://github.com/atmagopal/gqcnn.git
    
    catkin build -j8

    cd gqcnn
    ./scripts/downloads/download_example_data.sh
    ./scripts/downloads/models/download_models.sh

## Launch service

    workon gqcnn && source $HOME/cvbridge_build_ws/devel/setup.bash
    roslaunch gqcnn grasp_planning_service.launch model_name:=GQCNN-4.0-SUCTION

## Test with saved data
    
    workon gqcnn && source $HOME/cvbridge_build_ws/devel/setup.bash
    cd ~/cvbridge_build_ws/src/gqcnn
    python examples/policy_ros.py --depth_image data/examples/clutter/phoxi/dex-net_4.0/depth_0.npy --segmask data/examples/clutter/phoxi/dex-net_4.0/segmask_0.png --camera_intr data/calib/phoxi/phoxi.intr
