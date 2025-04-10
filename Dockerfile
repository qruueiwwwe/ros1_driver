FROM ros:noetic

# 安装必要的依赖
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-opencv \
    ros-noetic-cv-bridge \
    && rm -rf /var/lib/apt/lists/*

# 创建工作空间
WORKDIR /catkin_ws/src
COPY . /catkin_ws/src/

# 编译工作空间
WORKDIR /catkin_ws
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && catkin_make"

# 设置环境变量
ENV PYTHONPATH=/catkin_ws/devel/lib/python3/dist-packages:$PYTHONPATH
ENV LD_LIBRARY_PATH=/catkin_ws/devel/lib:$LD_LIBRARY_PATH

# 设置入口点
ENTRYPOINT ["/bin/bash", "-c", "source /opt/ros/noetic/setup.bash && source /catkin_ws/devel/setup.bash && roslaunch deviceshifu_driver deviceshifu_driver.launch"] 