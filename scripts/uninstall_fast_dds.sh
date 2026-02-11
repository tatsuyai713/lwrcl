#!/bin/bash


sudo rm -rf ~/Fast-DDS
sudo rm -rf ~/Fast-DDS-Gen
sudo rm -rf /opt/gradle

cat uninstall_info/fastcdr_install_manifest.txt | sudo xargs rm
cat uninstall_info/fastcdr_install_manifest.txt | xargs -L1 dirname | sudo xargs rmdir -p

cat uninstall_info/fastdds_install_manifest.txt | sudo xargs rm
cat uninstall_info/fastdds_install_manifest.txt | xargs -L1 dirname | sudo xargs rmdir -p

cat uninstall_info/foonathan_memory_vendor_install_manifest.txt | sudo xargs rm
cat uninstall_info/foonathan_memory_vendor_install_manifest.txt | xargs -L1 dirname | sudo xargs rmdir -p

cat uninstall_info/memory_install_manifest.txt | sudo xargs rm
cat uninstall_info/memory_install_manifest.txt | xargs -L1 dirname | sudo xargs rmdir -p

# sudo apt purge -y libasio-dev libtinyxml2-dev libssl-dev libp11-dev libengine-pkcs11-openssl softhsm2 libengine-pkcs11-openssl swig
# sudo apt autoremove -y

sed -i -e '/export LD_LIBRARY_PATH=\/usr\/local\/lib:$LD_LIBRARY_PATH/d' ~/.bashrc
sed -i -e '/export PATH=$PATH:\/opt\/gradle\/gradle-7.5.1\/bin/d' ~/.bashrc
sed -i -e '/export PATH=$PATH:$HOME\/Fast-DDS-Gen\/scripts/d' ~/.bashrc