#!/usr/bin/env bash
ABSOLUTE_PATH="/data/hj"

echo $#
if [ $# == 2 ]; then
  ABSOLUTE_PATH=$2
  echo ${ABSOLUTE_PATH}
fi

source /opt/ros/melodic/setup.bash
# export LD_LIBRARY_PATH=/${ABSOLUTE_PATH}/lib:${LD_LIBRARY_PATH}:/${ABSOLUTE_PATH}/lib/thirdparty
export LD_LIBRARY_PATH=/${ABSOLUTE_PATH}/lib:/${ABSOLUTE_PATH}/lib/slam_node/slam_node:/${ABSOLUTE_PATH}/lib/ziplib:${LD_LIBRARY_PATH}
export HJ_NODE_CONFIG_FILE=${ABSOLUTE_PATH}/config
export LOG_RECORDER_NEW_PATH="${ABSOLUTE_PATH}/config/log_recorder.json"
export BIG_DATA_CONFIG_FILE="${ABSOLUTE_PATH}/config/big_data_config.json"
#export HJ_ALL_LOG_CLOSE=close
#export HJ_LOG_CLOSE_collect_node=close
mkdir -p /tmp/logging
mkdir -p /userdata/hj/log/logging
rm -rf /userdata/hj/log/sensor_data

if [ ! -d "/userdata/hj/maps" ]; then
    mkdir -p /userdata/hj/maps
fi

# 设置基础路径文件夹
base_folder="/tmp/planning_log"

# 判断基础路径文件夹是否存在，如果不存在则创建
if [ ! -d "$base_folder" ]; then
	    mkdir -p "$base_folder"
fi

cd "$base_folder"
rm -rf *


# 使用当前时间创建子文件夹
sub_folder="$base_folder/$(date +%Y-%m-%d_%H-%M-%S)"
mkdir -p "$sub_folder"

# 将新创建的子文件夹路径设置为环境变量 HJ_uwr_LOG_PATH
export HJ_uwr_LOG_PATH="$sub_folder"

# 输出日志文件夹路径
echo "HJ_uwr_LOG_PATH is set to: $HJ_uwr_LOG_PATH"
rm -rf /dev/shm/MINOS_SHM
while true; do
    COUNT=$(ps -A | grep roscore | wc -l)
    if [ $COUNT -gt 1 ]; then
      break
    fi
    roscore &
    sleep 1 # wait roscore running
done


PID=`ps -ef | grep -E "monitor.sh|middleware_node|planning_node|slam_node|collect_node|utils_node|log_recorder|rosctl_ser|httpsrv" | grep -v grep | awk '{print $1}'`
for item  in $PID
do
	kill -s 9 $item
	echo "kill -s 9 $item"
done

PID=`ps -ef | grep -E "collect_node" | grep -v grep | awk '{print $1}'`
for item  in $PID
do
	kill -s 2 $item
	echo "kill -s 2 $item"
	IS_EXIST=`ps -ef | grep -E "collect_node" | grep -v grep`
	while [ -n "$IS_EXIST"  ]
	do
			echo "steal exit"
			sleep 1
            IS_EXIST=`ps -ef | grep -E "collect_node" | grep -v grep`
	done
done

#rosparam set /hj_so_path "${ABSOLUTE_PATH}/lib"
#rosparam set /hj_config_path "${ABSOLUTE_PATH}/config"
#./log_recorder > /dev/null &
START_LOG_FILE=/userdata/hj/log/logging/start.log
if [ -f "$START_LOG_FILE" ];then
  SIZE=`ls -l $START_LOG_FILE | awk {'print $5'}`
  if [ -n "$SIZE" ] && [ "$SIZE" -gt 16000 ];then
    rm -rf $START_LOG_FILE
  fi
fi

${ABSOLUTE_PATH}/bin/collect_node 2>&1 | tee -a $START_LOG_FILE > /dev/null &
sleep 2  # wait collect_node running, 顺序加载依赖库，防止i/o抢占导致初始化更慢
${ABSOLUTE_PATH}/bin/middleware_node 2>&1 | tee -a $START_LOG_FILE > /dev/null &
sleep 2
${ABSOLUTE_PATH}/bin/planning_node 2>&1 | tee -a $START_LOG_FILE > /dev/null &
${ABSOLUTE_PATH}/bin/slam_node 2>&1 | tee -a $START_LOG_FILE > /dev/null &
${ABSOLUTE_PATH}/bin/utils_node 2>&1 | tee -a $START_LOG_FILE > /dev/null &
rosctl_ser 13001 &
${ABSOLUTE_PATH}/bin/httpsrv &

# 守护进程
if [ -z "$1" ]; then
  KEEP_PROGRESS_FLAG="close"
else
  KEEP_PROGRESS_FLAG=$1
fi
if [ "$KEEP_PROGRESS_FLAG" == "open" -o "$KEEP_PROGRESS_FLAG" == "close" ]; then
    count1=`ps -ef | grep -E "keep_progress" | grep -v grep | wc -l`
    PID1=`ps -ef | grep -E "keep_progress.sh" | grep -v grep | awk '{print $1}'`
    if [ "$KEEP_PROGRESS_FLAG" == "open" -a $count1 -eq 0 ];then
        ${ABSOLUTE_PATH}/bin/keep_progress.sh > /dev/null &
    elif [ "$KEEP_PROGRESS_FLAG" == "close" -a -n "$PID1" ];then
        for item  in $PID1
        do
            kill -9 $item
            echo "kill -s 9 $item"
        done
    fi
else
    echo "keep_progress_flag error, please input open or close.  eg: ./hj_manager.sh open"
fi
