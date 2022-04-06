#!/bin/bash

# installed declarted ros packages
sudo apt install python3-rosdep python3-pip
rosdep update
rosdep install --from-paths . --ignore-src -r -y

# install pymavlink
sudo apt install python3-pip
pip install pymavlink --user

# install openvino
# source: https://docs.openvino.ai/latest/openvino_docs_install_guides_installing_openvino_apt.html
wget https://apt.repos.intel.com/intel-gpg-keys/GPG-PUB-KEY-INTEL-SW-PRODUCTS.PUB
sudo apt-key add GPG-PUB-KEY-INTEL-SW-PRODUCTS.PUB && rm GPG-PUB-KEY-INTEL-SW-PRODUCTS.PUB
echo "deb https://apt.repos.intel.com/openvino/2022 focal main" | sudo tee /etc/apt/sources.list.d/intel-openvino-2022.list
sudo apt update
sudo apt install openvino libngraph0-dev
SOURCE_OPENVINO="source /opt/intel/openvino_2022/setupvars.sh"
grep -qxF "$SOURCE_OPENVINO" ~/.bashrc || echo "$SOURCE_OPENVINO" >> ~/.bashrc
source ~/.bashrc
