#!/bin/bash

# プロセスIDを指定してください
PID=$1

# インターバル（秒単位）
INTERVAL=5

# ヘッダーの表示
echo -e "Timestamp\tMemory (KB)\tCPU (%)\tThread Count\tThread ID\tThread CPU (%)"

while true; do
    TIMESTAMP=$(date +"%Y-%m-%d %H:%M:%S")

    # メモリ使用量の取得（KB単位）
    MEMORY=$(pmap $PID | tail -n 1 | awk '/total/ {print $2}' | sed 's/K//')

    # プロセス全体のCPU負荷の取得
    CPU=$(ps -p $PID -o %cpu=)

    # スレッド数の取得
    THREAD_COUNT=$(ls /proc/$PID/task | wc -l)

    # スレッドごとのCPU負荷の取得
    THREAD_INFO=$(ps -p $PID -L -o tid,%cpu --no-headers)

    # メインプロセスの情報を表示
    echo -e "${TIMESTAMP}\t${MEMORY}\t${CPU}\t${THREAD_COUNT}"

    # 各スレッドの情報を表示
    # while IFS= read -r line; do
    #     THREAD_ID=$(echo $line | awk '{print $1}')
    #     THREAD_CPU=$(echo $line | awk '{print $2}')
    #     echo -e "${TIMESTAMP}\t${MEMORY}\t${CPU}\t${THREAD_COUNT}\t${THREAD_ID}\t${THREAD_CPU}"
    # done <<< "$THREAD_INFO"

    sleep $INTERVAL
done
