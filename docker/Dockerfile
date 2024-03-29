FROM ubuntu:20.04
LABEL maintainer="James Goppert <james.goppert@gmail.com>"

# environment
ENV XDG_RUNTIME_DIR=/tmp/runtime-docker
ENV NVIDIA_VISIBLE_DEVICES=all
ENV NVIDIA_DRIVER_CAPABILITIES=all
ENV TERM=xterm-256color
ENV DISPLAY=:20
ENV PATH="/home/user/bin:${PATH}"

# Set default shell during Docker image build to bash
SHELL ["/bin/bash", "-l", "-c"]

# Copy docker clean script
COPY install/docker_clean.sh /docker_clean.sh
RUN chmod +x /docker_clean.sh

# Install base packages
RUN DEBIAN_FRONTEND=noninteractive apt-get -y update && \
	apt-get -y upgrade && \
	apt-get install --no-install-recommends -y \
		sudo \
		locales \
		&& \
	/docker_clean.sh

# Initialise system locale
ENV LANG=en_US.UTF-8
ENV LANGUAGE=en_US:en
ENV LC_ALL=en_US.UTF-8
RUN locale-gen en_US.UTF-8

# Create a user to make sure install works without root
ARG UID_INSTALLER=2001
RUN useradd -l -u $UID_INSTALLER installer -G sudo,plugdev && \
 echo '%sudo ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers

USER installer

# add groups before we do anything that might add a new group
ARG GID_INPUT=107
ARG GID_RENDER=110
ARG GID_VGLUSERS=1005
RUN sudo groupadd -r -g $GID_INPUT input && \
 sudo groupadd -r -g $GID_RENDER render && \
 sudo groupadd -r -g $GID_VGLUSERS vglusers

# install dependencies using scripts in a manner that will cache build
# when one script is modified
COPY install/base.sh /tmp/install/base.sh
RUN /tmp/install/base.sh && /docker_clean.sh

COPY install/ros.sh /tmp/install/ros.sh
RUN /tmp/install/ros.sh && /docker_clean.sh

COPY install/gazebo.sh /tmp/install/gazebo.sh
RUN /tmp/install/gazebo.sh && /docker_clean.sh

# install virtualgl and turbovnc
COPY install/virtualgl_3.0.91_amd64.deb /tmp
RUN cd /tmp && \
  wget https://sourceforge.net/projects/turbovnc/files/3.0.2/turbovnc_3.0.2_amd64.deb  && \
  sudo apt-get -y update && \
  sudo apt-get -y upgrade && \
  sudo DEBIAN_FRONTEND=noninteractive  apt-get install --no-install-recommends -y \
    ./virtualgl_3.0.91_amd64.deb \
    ./turbovnc_3.0.2_amd64.deb && \
  /docker_clean.sh

COPY install/open_vino.sh /tmp/install/open_vino.sh
RUN /tmp/install/open_vino.sh && /docker_clean.sh

COPY install/install_geographiclib_datasets.sh /tmp/install/install_geographiclib_datasets.sh
RUN sudo /tmp/install/install_geographiclib_datasets.sh && /docker_clean.sh

COPY install/qgc.sh /tmp/install/qgc.sh
RUN /tmp/install/qgc.sh && /docker_clean.sh

COPY install/extra.sh /tmp/install/extra.sh
RUN /tmp/install/extra.sh && /docker_clean.sh

# enable apt auto-completion by deleting autoclean task
RUN sudo rm /etc/apt/apt.conf.d/docker-clean

# create a user for running the container
ARG UID_USER=1000
RUN sudo useradd --create-home -l -u $UID_USER -G sudo,plugdev,render,input,vglusers user && \
 echo user: $UID_USER && \
 sudo vglserver_config +egl

USER user

ARG VNCPASSWD=auav
COPY install/user_setup.sh /tmp/install/user_setup.sh
RUN /tmp/install/user_setup.sh $VNCPASSWD && /docker_clean.sh

# create setting directory for gazebo
VOLUME /home/user/.gz
RUN mkdir -p /home/user/.gz && \
  chown -R user:user /home/user/.gz

# create setting directory for qgroundcontrol
VOLUME /home/user/.config/QGroundControl.org
RUN mkdir -p /home/user/.config/QGroundControl.org && \
  chown -R user:user /home/user/.config/QGroundControl.org

# create workspace, this is where the source code will be mounted
VOLUME /home/user/ws
WORKDIR /home/user/ws
RUN sudo mkdir -p /home/user/ws && sudo chown -R user:user /home/user/ws

# create XDG runtime dir
RUN mkdir /tmp/runtime-docker && sudo chmod 700 /tmp/runtime-docker

# setup entry point
COPY install/entrypoint.sh /
RUN sudo chmod +x /entrypoint.sh

CMD ["/bin/bash"]
ENTRYPOINT ["/entrypoint.sh"]

LABEL org.opencontainers.image.source = "https://github.com/jgoppert/auav_2022_sample"

# vim: set et fenc=utf-8 ff=unix ft=dockerfile sts=0 sw=2 ts=2 :
