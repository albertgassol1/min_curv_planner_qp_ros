# Use Ubuntu 22.04 as the base image
FROM ubuntu:22.04

# Set environment to non-interactive (no prompts during installation)
ENV DEBIAN_FRONTEND=noninteractive

# Update the package list and install essential build tools and libraries
RUN apt-get update && apt-get install -y \
    build-essential cmake make git curl \
    wget nano clang clang-format clang-tidy \         
    python3 python3-pip libeigen3-dev libboost-all-dev \
    python3-tk libgl1-mesa-glx libxrender1 libsm6 \
    libxext6 libyaml-cpp-dev && apt-get clean && rm -rf /var/lib/apt/lists/* \
    apt-get update

# Set up a default working directory
WORKDIR /workspace

# Install osqp and osqp-eigen
RUN cd /workspace && mkdir third_party && cd third_party && \
    git clone https://github.com/oxfordcontrol/osqp && \
    cd osqp && mkdir build && cd build && \
    cmake .. -G "Unix Makefiles" -DCMAKE_BUILD_TYPE=Release && \
    make && make install && \
    cd /workspace/third_party && \ 
    git clone https://github.com/robotology/osqp-eigen && \
    cd osqp-eigen && mkdir build && cd build && \
    cmake .. -G "Unix Makefiles" -DCMAKE_BUILD_TYPE=Release && \
    make && make install
# Install pip and python packages
RUN pip3 install 'numpy==1.24.4' matplotlib pandas fire

# Copy the files
COPY . ./min_curv_planner_qp

# Build the project
RUN cd /workspace/min_curv_planner_qp && mkdir build && mkdir bin && cd build && \
    cmake .. -G "Unix Makefiles" -DCMAKE_BUILD_TYPE=Release && \
    make 

# Remove the files
RUN rm -rf /workspace/min_curv_planner_qp/examples && \
    rm -rf /workspace/min_curv_planner_qp/min_curv_lib &&\
    rm -rf /workspace/min_curv_planner_qp/CMakeLists.txt &&\
    rm -rf /workspace/min_curv_planner_qp/README.md &&\
    rm -rf /workspace/min_curv_planner_qp/Dockerfile &&\
    rm -rf /workspace/min_curv_planner_qp/.gitignore
