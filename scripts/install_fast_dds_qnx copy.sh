#!/bin/bash

QNX_PATH="qnx800"
fast_dds_version="2.14.2"
foonathan_memory_vendor_version="1.3.1"
googletest_version="1.13.0"
fast_dds_gen_version="3.3.0"

FAST_DDS_WORK_DIR=./dds_build

if [ $# != 1 ]; then
    echo "Please select QNX path."
    exit 1
fi

INSTALL_PATH=/opt/qnx/fast-dds/
sudo mkdir -p $INSTALL_PATH
sudo chmod 777 -R ${INSTALL_PATH}

source ~/$QNX_PATH/qnxsdp-env.sh

FAST_DDS_WORK_DIR=./dds_build

sudo apt install -y dos2unix

sudo rm -rf ${FAST_DDS_WORK_DIR}

mkdir ${FAST_DDS_WORK_DIR}

cd ${FAST_DDS_WORK_DIR}
git clone https://github.com/eProsima/Fast-DDS.git -b v$fast_dds_version --depth 1
cd Fast-DDS
WORKSPACE=$PWD
git apply $WORKSPACE/../../patches/fastdds-qnx.patch
git submodule update --init $PWD/thirdparty/asio $PWD/thirdparty/fastcdr $PWD/thirdparty/tinyxml2
cd $WORKSPACE/thirdparty/asio
git apply $WORKSPACE/build_qnx/qnx_patches/asio_qnx.patch
cd $WORKSPACE/thirdparty/fastcdr
git apply $WORKSPACE/build_qnx/qnx_patches/fastcdr_qnx.patch
cd $WORKSPACE/thirdparty/tinyxml2
unix2dos $WORKSPACE/build_qnx/qnx_patches/tinyxml2_qnx.patch
git apply $WORKSPACE/build_qnx/qnx_patches/tinyxml2_qnx.patch

cd ${WORKSPACE}
git clone https://github.com/eProsima/foonathan_memory_vendor.git -b v$foonathan_memory_vendor_version

# Exclude googletest
cd ${WORKSPACE}
sed -i '/googletest/s/^/#/g' build_qnx/common.mk

cd ${WORKSPACE}/build_qnx
make INSTALL_ROOT_nto="${INSTALL_PATH}"
make install INSTALL_ROOT_nto="${INSTALL_PATH}"

## TODO: Fix tf2 build error
## sudo mkdir -p /opt/qnx/fast-dds/aarch64le/usr/include/fastdds/thirdparty
## sudo ln -s /opt/qnx/fast-dds-libs/include/optionparser /opt/qnx/fast-dds/aarch64le/usr/include/fastdds/thirdparty/optionparser

# Java packages for FastDDS Generator and other similar tools
sudo mkdir -p /usr/share/man/man1
sudo apt update 
sudo apt install -y -q --no-install-recommends openjdk-11-jdk-headless

# Install Fast-DDS-Gen
# This requires Gradle (build system for Java) which we can install in the same temporary location
sudo rm -rf /opt/fast-dds-gen
sudo mkdir -p /opt/fast-dds-gen
sudo rm -rf /tmp/ddsgen-build && mkdir /tmp/ddsgen-build && cd /tmp/ddsgen-build \
    && wget https://services.gradle.org/distributions/gradle-7.5.1-bin.zip \
    && unzip gradle-7.5.1-bin.zip \
    && git clone --depth 1 -b v$fast_dds_gen_version https://github.com/eProsima/Fast-DDS-Gen.git fast-dds-gen \
    && cd fast-dds-gen \
    && ../gradle-7.5.1/bin/gradle assemble \
    && sudo ../gradle-7.5.1/bin/gradle install --install_path=/opt/fast-dds-gen \
    && sudo mv ../gradle-7.5.1 /opt/fast-dds-gen/src/ \
    && sudo rm -rf /tmp/ddsgen-build

set +e
grep 'export PATH=$PATH:/opt/gradle/gradle-7.5.1/bin' ~/.bashrc
if [ $? = 0 ]; then
        echo "PATH is already added"
else
        echo 'export PATH=$PATH:/opt/gradle/gradle-7.5.1/bin' >> ~/.bashrc
        source ~/.bashrc
fi
grep 'export PATH=$PATH:/opt/fast-dds-gen/bin' ~/.bashrc
if [ $? = 0 ]; then
        echo "PATH is already added"
else
        echo 'export PATH=$PATH:/opt/fast-dds-gen/bin' >> ~/.bashrc
        source ~/.bashrc
fi


qnx_path="qnx800"
fast_dds_version="2.14.2"
foonathan_memory_vendor_version="1.3.1"
googletest_version="1.13.0"
fast_dds_gen_version="3.3.0"

FAST_DDS_WORK_DIR=./dds_build

source ~/qnx800/qnxsdp-env.sh

wget -O - https://apt.kitware.com/keys/kitware-archive-latest.asc 2>/dev/null | gpg --dearmor - | sudo tee /etc/apt/trusted.gpg.d/kitware.gpg >/dev/null

sudo apt-add-repository "deb https://apt.kitware.com/ubuntu/ $(lsb_release -cs) main"

sudo apt update
sudo apt install -y gcc g++ make cmake automake autoconf unzip git vim openssl gcc make cmake curl tar wget p11-kit

p11-kit list-modules

openssl engine pkcs11 -t

sudo rm -rf ${FAST_DDS_WORK_DIR}

mkdir ${FAST_DDS_WORK_DIR}

cd ${FAST_DDS_WORK_DIR}
git clone https://github.com/eProsima/Fast-DDS.git -b v$fast_dds_version --depth 1
cd Fast-DDS
WORKSPACE=$PWD
git submodule update --init $PWD/thirdparty/asio $PWD/thirdparty/fastcdr $PWD/thirdparty/tinyxml2
cd ${WORKSPACE}
git clone https://github.com/eProsima/foonathan_memory_vendor.git -b v$foonathan_memory_vendor_version
cd ${WORKSPACE}
git clone https://github.com/google/googletest.git -b v$googletest_version --depth 1

INSTALL_PATH=/opt/fast-dds
sudo rm -rf $INSTALL_PATH
sudo mkdir $INSTALL_PATH

cd $WORKSPACE
cd foonathan_memory_vendor && mkdir build && cd build &&\
CXXFLAGS="-O3" cmake -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX=$INSTALL_PATH ../ && \
make -j4 && sudo make install

cd $WORKSPACE
cd googletest && mkdir build && cd build &&\
CXXFLAGS="-O3" cmake -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX=$INSTALL_PATH ../ && \
make -j4 && sudo make install

# Build and install Fast-CDR (required for FastDDS)
cd $WORKSPACE
cd thirdparty/fastcdr && mkdir build && cd build && \
CXXFLAGS="-O3" cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=$INSTALL_PATH ../ &&
make -j4 && sudo make install


# Build and install TinyXML2 (required for FastDDS)
cd $WORKSPACE
cd thirdparty/tinyxml2 && mkdir build && cd build && \
CXXFLAGS="-O3 -fPIC" cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=$INSTALL_PATH ../ &&
make -j4 && sudo make install


# Build and install ASIO (requited for FastDDS)
cd $WORKSPACE
cd thirdparty/asio/asio && \
./autogen.sh && \
./configure CXXFLAGS="-O3 -g -DASIO_HAS_PTHREADS -D_GLIBCXX_HAS_GTHREADS -std=c++11" --prefix=$INSTALL_PATH  && \
make -j4 && sudo make install


# Build and install Fast-DDS
cd $WORKSPACE
rm -rf ./build && mkdir build && cd build && \
CXXFLAGS="-DASIO_HAS_PTHREADS=1" \
  cmake -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX=$INSTALL_PATH \
  -Dfastcdr_DIR=$INSTALL_PATH/lib/cmake/fastcdr/ \
  -Dfoonathan_memory_DIR=$INSTALL_PATH/lib/foonathan_memory/cmake/ \
  -DCMAKE_SYSTEM_PREFIX_PATH=$INSTALL_PATH \
  -DCMAKE_PREFIX_PATH=$INSTALL_PATH \
  .. && \
make -j4 && sudo make install

sed -i -e '/export PATH=$PATH:\/opt\/fast-dds\/bin/d' ~/.bashrc
echo 'export PATH=$PATH:/opt/fast-dds/bin' >> ~/.bashrc

if grep 'export LD_LIBRARY_PATH=/opt/fast-dds/lib:$LD_LIBRARY_PATH' ~/.bashrc >/dev/null; then
  echo "LD_LIBRARY_PATH libs are already added"
else
  echo 'export LD_LIBRARY_PATH=/opt/fast-dds/lib:$LD_LIBRARY_PATH' >> ~/.bashrc
  source ~/.bashrc
fi
sudo ldconfig
