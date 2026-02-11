#!/bin/bash

QNX_IP=192.168.23.4

if [ "$#" -ne 2 ]; then
  echo "Please select one execution file and qnx path."
  exit 1
fi

executable_path="$1"

rm -rf ./deploy_files
mkdir deploy_files
destination_folder=./deploy_files

search_folder1="/opt/qnx/cyclonedds/"
search_folder2="/opt/qnx/cyclonedds-libs/"
search_folder3="$2/target/qnx/aarch64le/"

dependencies=$(objdump -p "$executable_path" | grep NEEDED | awk '{print $2}')

echo "$dependencies"

for dep in $dependencies; do
  # 最初の検索フォルダでファイルを検索
  find "$search_folder1" -name "$dep" -exec bash -c 'file="{}"; if [ -L "$file" ]; then cp -vP "$file" "'$destination_folder'"; realfile=$(readlink -f "$file"); cp -v "$realfile" "'$destination_folder'"; else cp -v "$file" "'$destination_folder'"; fi' \;
  
  # 2番目の検索フォルダでファイルを検索
  find "$search_folder2" -name "$dep" -exec bash -c 'file="{}"; if [ -L "$file" ]; then cp -vP "$file" "'$destination_folder'"; realfile=$(readlink -f "$file"); cp -v "$realfile" "'$destination_folder'"; else cp -v "$file" "'$destination_folder'"; fi' \;

  # 3番目の検索フォルダでファイルを検索
  find "$search_folder3" -name "$dep" -exec bash -c 'file="{}"; if [ -L "$file" ]; then cp -vP "$file" "'$destination_folder'"; realfile=$(readlink -f "$file"); cp -v "$realfile" "'$destination_folder'"; else cp -v "$file" "'$destination_folder'"; fi' \;
done
  

find "$destination_folder" -type f -o -type l > /tmp/temp_file_list.txt

execution_file=$(basename "$executable_path")
echo "cd /data/home/qnxuser/qnx" > ./deploy_qnx_batchfile
echo "cd $execution_file" >> ./deploy_qnx_batchfile
echo "put $executable_path" >> ./deploy_qnx_batchfile
# echo "put ./fastdds.xml" >> ./deploy_qnx_batchfile

# 一時ファイルを読み込み、各ファイルの前に "put " を追加して最終的なファイルに書き出す
while read -r line; do
    echo "put $line"
done < /tmp/temp_file_list.txt >> ./deploy_qnx_batchfile

# 一時ファイルを削除
rm /tmp/temp_file_list.txt

folder_path=$(dirname "$executable_path")
config_dir="$folder_path/config"
# config フォルダが存在するかどうかを確認
if [ -d "$config_dir" ]; then
  find "$config_dir" -type f -o -type l > /tmp/temp_config_list.txt
  echo "mkdir config" >> ./deploy_qnx_batchfile
  echo "cd config" >> ./deploy_qnx_batchfile
  while read -r line; do
    echo "put $line"
  done < /tmp/temp_config_list.txt >> ./deploy_qnx_batchfile
  rm /tmp/temp_config_list.txt
fi

SSH_PASSWORD="qnxuser"

sshpass -p "$SSH_PASSWORD" ssh qnxuser@$QNX_IP "mkdir -p /data/home/qnxuser/qnx/$execution_file"
sshpass -p "$SSH_PASSWORD" ssh qnxuser@$QNX_IP "rm -rf /data/home/qnxuser/qnx/$execution_file"
sshpass -p "$SSH_PASSWORD" ssh qnxuser@$QNX_IP "mkdir -p /data/home/qnxuser/qnx/$execution_file"

sshpass -p "$SSH_PASSWORD" sftp -oBatchMode=no -b ./deploy_qnx_batchfile qnxuser@$QNX_IP:/data/home/qnxuser/qnx/
