#!/bin/bash

QNX_PATH="qnx800"
fast_dds_version="2.11.2"
foonathan_memory_vendor_version="1.3.1"
googletest_version="1.13.0"

FAST_DDS_WORK_DIR=./dds_build

wget -O - https://apt.kitware.com/keys/kitware-archive-latest.asc 2>/dev/null | gpg --dearmor - | sudo tee /etc/apt/trusted.gpg.d/kitware.gpg >/dev/null

sudo apt-add-repository "deb https://apt.kitware.com/ubuntu/ $(lsb_release -cs) main"

sudo apt update
sudo apt install -y gcc g++ make cmake automake autoconf unzip git vim openssl gcc make cmake=3.31.8* cmake-data=3.31.8* curl tar wget p11-kit
sudo apt-mark hold cmake cmake-data

p11-kit list-modules

openssl engine pkcs11 -t

INSTALL_PATH=/opt/qnx/fast-dds/
sudo mkdir -p $INSTALL_PATH
sudo chmod 777 -R ${INSTALL_PATH}

source ~/$QNX_PATH/qnxsdp-env.sh

sudo apt install -y dos2unix

sudo rm -rf ${FAST_DDS_WORK_DIR}

mkdir ${FAST_DDS_WORK_DIR}

cd ${FAST_DDS_WORK_DIR}
git clone https://github.com/eProsima/Fast-DDS.git -b v$fast_dds_version --depth 1
cd Fast-DDS
WORKSPACE=$PWD
echo "Patch Fast-DDS"
git apply $WORKSPACE/../../patches/fastdds-qnx.patch
echo "Patch Fast-DDS done"
git submodule update --init $PWD/thirdparty/asio $PWD/thirdparty/fastcdr $PWD/thirdparty/tinyxml2
cd $WORKSPACE/thirdparty/asio
echo "Patch Asio"
git apply $WORKSPACE/build_qnx/qnx_patches/asio_qnx.patch
echo "Patch Asio done"
cd $WORKSPACE/thirdparty/fastcdr
echo "Patch FastCDR"
git apply $WORKSPACE/build_qnx/qnx_patches/fastcdr_qnx.patch
echo "Patch FastCDR done"
cd $WORKSPACE/thirdparty/tinyxml2
unix2dos $WORKSPACE/build_qnx/qnx_patches/tinyxml2_qnx.patch
echo "Patch TinyXML2"
git apply $WORKSPACE/build_qnx/qnx_patches/tinyxml2_qnx.patch
echo "Patch TinyXML2 done"

cd ${WORKSPACE}
git clone https://github.com/eProsima/foonathan_memory_vendor.git -b v$foonathan_memory_vendor_version
cd $WORKSPACE
git clone https://github.com/google/googletest.git && cd googletest
git checkout v$googletest_version
git apply $WORKSPACE/build_qnx/qnx_patches/googletest_qnx.patch

cd ${WORKSPACE}/build_qnx
make INSTALL_ROOT_nto="${INSTALL_PATH}"
make install INSTALL_ROOT_nto="${INSTALL_PATH}"

