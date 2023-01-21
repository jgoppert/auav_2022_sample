#!/bin/bash
set -e

source ~/.profile
source ~/.bashrc

if [ "$RUN_VNC" = true ] ; then
  /opt/TurboVNC/bin/vncserver -geometry 1920x1080 -name auav -xstartup /bin/openbox-session :20
fi

exec "$@"
