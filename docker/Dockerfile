FROM osrf/ros:humble-desktop

SHELL ["/bin/bash", "-c"]

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=humble
ENV WS=/ros2_ws

# Basic tools + ROS dev tooling + zsh
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    python3-argcomplete \
    python3-colcon-argcomplete \
    ros-humble-rmw-cyclonedds-cpp \
    zsh \
    git \
    git-lfs \
    build-essential \
    cmake \
    gdb \
    vim \
    nano \
    curl \
    wget \
    ca-certificates \
    locales \
    software-properties-common \
 && rm -rf /var/lib/apt/lists/*

# Locale
RUN locale-gen en_US en_US.UTF-8 \
 && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
ENV LANG=en_US.UTF-8
ENV LC_ALL=en_US.UTF-8

# rosdep
RUN if [ ! -e /etc/ros/rosdep/sources.list.d/20-default.list ]; then \
      rosdep init; \
    fi \
 && rosdep update

# Workspace
RUN mkdir -p ${WS}/src
WORKDIR ${WS}

# Zsh configuration (ROS sourcing + useful defaults)
RUN sh -c "$(curl -fsSL https://raw.githubusercontent.com/ohmyzsh/ohmyzsh/master/tools/install.sh)" "" --unattended \
 && sed -i 's/ZSH_THEME=".*"/ZSH_THEME="robbyrussell"/' /root/.zshrc
 RUN echo 'autoload -U compinit && compinit' >> /root/.zshrc \
 && echo 'source /opt/ros/${ROS_DISTRO}/setup.zsh' >> /root/.zshrc \
 && echo '[ -f ${WS}/install/setup.zsh ] && source ${WS}/install/setup.zsh' >> /root/.zshrc \
 && echo 'eval "$(register-python-argcomplete3 ros2)"' >> /root/.zshrc \
 && echo 'eval "$(register-python-argcomplete3 colcon)"' >> /root/.zshrc \
 && echo '[ -f /usr/share/colcon_argcomplete/hook/colcon-argcomplete.zsh ] && source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.zsh' >> /root/.zshrc \
 && echo 'export ROS_DOMAIN_ID=0' >> /root/.zshrc \
 && echo 'export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp' >> /root/.zshrc
 

# Also keep bash working (fallback)
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /root/.bashrc \
 && echo "[ -f ${WS}/install/setup.bash ] && source ${WS}/install/setup.bash" >> /root/.bashrc

# Set zsh as default shell
CMD ["zsh"]