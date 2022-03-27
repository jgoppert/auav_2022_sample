#!/bin/bash
sudo apt install python3-rosdep
rosdep install --from-paths . --ignore-src -r -y
