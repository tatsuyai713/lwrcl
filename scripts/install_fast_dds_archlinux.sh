#!/bin/bash

fast_dds_version="2.11.2"
foonathan_memory_vendor_version="1.3.1"
googletest_version="1.13.0"
fast_dds_gen_version="2.4.0"

FAST_DDS_WORK_DIR=./dds_build

sudo pacman -Sy
sudo pacman -S gcc make cmake automake autoconf unzip git vim openssl gcc make cmake curl tar jre11-openjdk jdk11-openjdk wget
sudo pacman -S community/opensc community/libp11
sudo archlinux-java set java-11-openjdk

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


cd $WORKSPACE
# Java packages for FastDDS Generator and other similar tools
sudo mkdir -p /usr/share/man/man1

sudo rm -rf /opt/gradle/
sudo mkdir /opt/gradle
cd /opt/gradle
sudo wget https://services.gradle.org/distributions/gradle-7.5.1-bin.zip
sudo unzip gradle-7.5.1-bin.zip
sudo rm -f gradle-7.5.1-bin.zip

sed -i -e '/export PATH=$PATH:\/opt\/gradle\/gradle-7.5.1\/bin/d' ~/.bashrc
echo 'export PATH=$PATH:/opt/gradle/gradle-7.5.1/bin' >> ~/.bashrc

sed -i -e '/export JAVA_HOME=\/usr\/lib\/jvm\/default/d' ~/.bashrc
echo 'export JAVA_HOME=/usr/lib/jvm/default' >> ~/.bashrc

export JAVA_HOME=/usr/lib/jvm/default
export PATH=$PATH:/opt/gradle/gradle-7.5.1/bin

# Install Fast-DDS-Gen
cd $WORKSPACE
sudo rm -rf /opt/fast-dds-gen
sudo mkdir -p /opt/fast-dds-gen
git clone --recursive -b v$fast_dds_gen_version https://github.com/eProsima/Fast-DDS-Gen.git fast-dds-gen \
    && cd fast-dds-gen \
    && gradle assemble \
    && sudo /opt/gradle/gradle-7.5.1/bin/gradle install --install_path=/opt/fast-dds-gen


sed -i -e '/export PATH=$PATH:\/opt\/fast-dds\/bin/d' ~/.bashrc
echo 'export PATH=$PATH:/opt/fast-dds/bin' >> ~/.bashrc

sed -i -e '/export PATH=$PATH:\/opt\/fast-dds-gen\/bin/d' ~/.bashrc
echo 'export PATH=$PATH:/opt/fast-dds-gen/bin' >> ~/.bashrc

if grep 'export LD_LIBRARY_PATH=/opt/fast-dds/lib:$LD_LIBRARY_PATH' ~/.bashrc >/dev/null; then
  echo "LD_LIBRARY_PATH libs are already added"
else
  echo 'export LD_LIBRARY_PATH=/opt/fast-dds/lib:$LD_LIBRARY_PATH' >> ~/.bashrc
  source ~/.bashrc
fi
sudo ldconfig
