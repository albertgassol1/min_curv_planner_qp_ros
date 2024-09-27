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

# Install cmake 3.18
RUN apt update && apt install -y wget && \
    wget https://github.com/Kitware/CMake/releases/download/v3.20.0/cmake-3.20.0.tar.gz && \
    tar -zxvf cmake-3.20.0.tar.gz && \
    cd cmake-3.20.0 && \
    ./bootstrap && \
    make && \
    make install


# Install osqp and osqp-eigen
RUN mkdir -p /root/third_party && cd /root/third_party && \
    git clone https://github.com/oxfordcontrol/osqp && \
    cd osqp && mkdir build && cd build && \
    cmake .. -G "Unix Makefiles" -DCMAKE_BUILD_TYPE=Release && \
    make && sudo make install && \
    cd /root/third_party && \ 
    git clone https://github.com/robotology/osqp-eigen && \
    cd osqp-eigen && mkdir build && cd build && \
    cmake .. -G "Unix Makefiles" -DCMAKE_BUILD_TYPE=Release && \
    make && sudo make install


# Install pip and python packages
RUN apt update && apt install -y python3-pip && \
    pip3 install 'numpy==1.24.4' matplotlib pandas fire


# Set the working directory
WORKDIR /root

# Copy the files
COPY . .
