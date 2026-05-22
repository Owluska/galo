FROM osrf/ros:humble-desktop

SHELL ["/bin/bash", "-c"]

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=humble
ENV WS=/ros2_ws
ENV LANG=en_US.UTF-8
ENV LC_ALL=en_US.UTF-8
ENV ROS_DOMAIN_ID=0
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    ca-certificates \
    cmake \
    curl \
    gdb \
    git \
    git-lfs \
    iproute2 \
    less \
    locales \
    nano \
    net-tools \
    python3-argcomplete \
    python3-colcon-argcomplete \
    python3-colcon-common-extensions \
    python3-pytest \
    python3-rosdep \
    python3-vcstool \
    software-properties-common \
    tcpdump \
    vim \
    wget \
    zsh \
    libeigen3-dev \
    libgeographic-dev \
    libpcap-dev \
    libpcl-dev \
    libyaml-cpp-dev \
    ros-${ROS_DISTRO}-gps-msgs \
    ros-${ROS_DISTRO}-nmea-msgs \
    ros-${ROS_DISTRO}-pcl-conversions \
    ros-${ROS_DISTRO}-pcl-ros \
    ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
    ros-${ROS_DISTRO}-robot-localization \
    ros-${ROS_DISTRO}-rosidl-default-generators \
    ros-${ROS_DISTRO}-tf2-geometry-msgs \
    ros-${ROS_DISTRO}-tf2-ros \
    ros-${ROS_DISTRO}-tf2-sensor-msgs \
 && rm -rf /var/lib/apt/lists/*

RUN locale-gen en_US en_US.UTF-8 \
 && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

RUN if [ ! -e /etc/ros/rosdep/sources.list.d/20-default.list ]; then \
      rosdep init; \
    fi \
 && rosdep update

RUN mkdir -p ${WS}/src
WORKDIR ${WS}

ARG INSTALL_OH_MY_ZSH=true
RUN if [ "${INSTALL_OH_MY_ZSH}" = "true" ]; then \
      sh -c "$(curl -fsSL https://raw.githubusercontent.com/ohmyzsh/ohmyzsh/master/tools/install.sh)" "" --unattended; \
      sed -i 's/ZSH_THEME=".*"/ZSH_THEME="robbyrussell"/' /root/.zshrc; \
    else \
      touch /root/.zshrc; \
    fi

RUN { \
      echo 'autoload -U compinit && compinit'; \
      echo 'source /opt/ros/${ROS_DISTRO}/setup.zsh'; \
      echo '[ -f ${WS}/install/setup.zsh ] && source ${WS}/install/setup.zsh'; \
      echo 'eval "$(register-python-argcomplete3 ros2)"'; \
      echo 'eval "$(register-python-argcomplete3 colcon)"'; \
      echo '[ -f /usr/share/colcon_argcomplete/hook/colcon-argcomplete.zsh ] && source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.zsh'; \
      echo 'export ROS_DOMAIN_ID=${ROS_DOMAIN_ID}'; \
      echo 'export RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION}'; \
    } >> /root/.zshrc

RUN { \
      echo 'source /opt/ros/${ROS_DISTRO}/setup.bash'; \
      echo '[ -f ${WS}/install/setup.bash ] && source ${WS}/install/setup.bash'; \
      echo 'export ROS_DOMAIN_ID=${ROS_DOMAIN_ID}'; \
      echo 'export RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION}'; \
    } >> /root/.bashrc

# Uncomment these lines if you build the image with this repository as context
# and want the workspace compiled into the image instead of bind-mounted.
# COPY . ${WS}/src
# RUN source /opt/ros/${ROS_DISTRO}/setup.bash \
#  && rosdep install --from-paths ${WS}/src --ignore-src -r -y \
#  && colcon build --symlink-install

CMD ["zsh"]
