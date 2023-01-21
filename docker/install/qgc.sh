#!/bin/bash
set -e

sudo apt-get -y update
sudo apt-get -y upgrade
sudo apt-get remove modemmanager -y
sudo DEBIAN_FRONTEND=noninteractive apt-get install --no-install-recommends -y \
	gstreamer1.0-plugins-bad \
	gstreamer1.0-libav \
	gstreamer1.0-gl \
	libqt5gui5 \
	libfuse3-3 \
	fuse \
	libcanberra-gtk-module \
	libpulse-mainloop-glib0 \
	ca-certificates

# download qgroundcontrol
cd /opt
sudo wget https://d176tv9ibo4jno.cloudfront.net/latest/QGroundControl.AppImage -O /opt/qgroundcontrol
sudo chmod +x /opt/qgroundcontrol
