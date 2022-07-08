#!/bin/bash

sleep 10
# 添加本地执行路径
stop(){
    echo "停止buff"
    ps -efww | grep -w 'buff' | grep -v grep | cut -c 9-16 | xargs kill -9
}

while true; do
    # 启动一个循环，定时检查进程是否存在
    server=$(ps -e | grep buff | grep -v grep)
    echo $server
    if [ ! "$server" ]; then
        echo "buff掉线,重新启动"
        # 如果不存在就重新启动
        nohup buff > /home/"$(uname -n)"/log/buff.log &
        # 启动后沉睡10s
        sleep 10
    else
        echo "buff存活"
    fi
    # 每次循环沉睡10s
    sleep 10
done