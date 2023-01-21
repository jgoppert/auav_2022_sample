#!/bin/bash
set -e

# install openvino
# source: https://docs.openvino.ai/latest/openvino_docs_install_guides_installing_openvino_apt.html
cd /tmp
wget https://apt.repos.intel.com/intel-gpg-keys/GPG-PUB-KEY-INTEL-SW-PRODUCTS.PUB
sudo apt-key add GPG-PUB-KEY-INTEL-SW-PRODUCTS.PUB && rm GPG-PUB-KEY-INTEL-SW-PRODUCTS.PUB
echo "deb https://apt.repos.intel.com/openvino/2022 focal main" | sudo tee /etc/apt/sources.list.d/intel-openvino-2022.list
sudo apt-get -y update
sudo apt-get -y upgrade
sudo DEBIAN_FRONTEND=noninteractive apt-get install --no-install-recommends -y \
	openvino libngraph0-dev
