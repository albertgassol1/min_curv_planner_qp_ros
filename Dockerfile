FROM osrf/ros:noetic-desktop-full

# Update all packages
RUN apt update && apt upgrade -y && apt install -y git

# Rosdep update
RUN rosdep update

# Source the ROS setup file
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc

################################
## ADD ANY CUSTOM SETUP BELOW ##
################################

WORKDIR /workspace

# Install cmake 3.18
RUN apt update && apt install -y wget && \
    wget https://github.com/Kitware/CMake/releases/download/v3.20.0/cmake-3.20.0.tar.gz && \
    tar -zxvf cmake-3.20.0.tar.gz && \
    cd cmake-3.20.0 && \
    ./bootstrap && \
    make && \
    make install

# Link Eigen3
RUN cd /usr/include && ln -s eigen3/Eigen Eigen && ln -s eigen3/unsupported unsupported


# Install osqp, osqp-eigen and catkin_simple
RUN mkdir -p /workspace/third_party && cd /workspace/third_party && \
    git clone https://github.com/oxfordcontrol/osqp && cd osqp &&\
    git checkout release-0.6.3 && \
    git submodule update --init --recursive && \
    mkdir build && cd build && \
    cmake .. -G "Unix Makefiles" -DCMAKE_BUILD_TYPE=Release && \
    make && make install && \
    cd /workspace/third_party && \ 
    git clone https://github.com/robotology/osqp-eigen && \
    cd osqp-eigen && mkdir build && cd build && \
    cmake .. -G "Unix Makefiles" -DCMAKE_BUILD_TYPE=Release && \
    make && make install

# Install pip and python packages
RUN apt update && apt install -y python3-pip && \
    pip3 install catkin_tools 'numpy==1.24.4' matplotlib pandas fire scipy

# Set the working directory
RUN mkdir -p /workspace/min_curv_planner_ws/src