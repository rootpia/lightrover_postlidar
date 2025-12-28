FROM ros:melodic-ros-base-bionic

# Install pip for Python 2.7
RUN apt-get update && apt-get install -y \
    python-pip \
    && rm -rf /var/lib/apt/lists/*

# Install python dependencies
RUN pip install requests mock

# Create workspace directory
WORKDIR /catkin_ws

# Setup ROS environment
RUN echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
