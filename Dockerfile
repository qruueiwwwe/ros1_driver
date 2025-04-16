FROM ros:noetic

# 安装必要的依赖
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-opencv \
    curl \
    ros-noetic-cv-bridge \
    && rm -rf /var/lib/apt/lists/*

RUN pip3 install flask 
# 创建工作空间
WORKDIR /catkin_ws/src
COPY . /catkin_ws/src/

# 编译工作空间
WORKDIR /catkin_ws
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && catkin_make"

# 设置环境变量
ENV PYTHONPATH=/catkin_ws/devel/lib/python3/dist-packages:$PYTHONPATH
ENV LD_LIBRARY_PATH=/catkin_ws/devel/lib:$LD_LIBRARY_PATH
ENV ROS_PACKAGE_PATH=/opt/ros/noetic/share:/catkin_ws/src:$ROS_PACKAGE_PATH
ENV ROS_MASTER_URI=http://192.168.31.101:11311
ENV PATH=/opt/ros/noetic/bin:$PATH

# 创建启动脚本
RUN echo '#!/bin/bash\n\
source /opt/ros/noetic/setup.bash\n\
source /catkin_ws/devel/setup.bash\n\
exec "$@"' > /entrypoint.sh && chmod +x /entrypoint.sh

# 设置入口点
ENTRYPOINT ["/entrypoint.sh"]
CMD ["roslaunch", "deviceshifu_driver", "deviceshifu_driver.launch"]
