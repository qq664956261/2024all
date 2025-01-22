#!/usr/bin/env bash
node_path="../../../../../"
echo $#
if [ $# == 1 ]; then
  
  node_path=$1
  echo ${node_path}
fi
mkdir /tmp/log

export HJ_NODE_CONFIG_FILE=${node_path}/src
export LOG_RECORDER_NEW_PATH=${node_path}/src/hj_manager/x9/config/amd64/log_recorder.json
export BIG_DATA_CONFIG_FILE=${node_path}/src/hj_manager/x9/config/amd64/big_data_config.json
export LD_LIBRARY_PATH=${node_path}/src/hj_interface/platforms/amd64:${node_path}/src/thirdparty/platforms/amd64:${node_path}/src/thirdparty/platforms/amd64/awsiot:${LD_LIBRARY_PATH}
rm -rf /dev/shm/MINOS_SHM
echo $LD_LIBRARY_PATH
while true; do
    C0UNT=$(ps -A | grep roscore | wc -l)
    if [ "$C0UNT" != "0" ]; then
        echo "get roscore"
        break
    fi
    roscore &
    echo "need sleep"
    sleep 1
done

PID=`ps -ef | grep -E "collect_node|utils_node" | grep -v grep | awk '{print $2}'`
for item  in $PID
do
	kill -s 2 $item
	echo "kill -s 2 $item"
	IS_EXIST=`ps -ef | grep -E "collect_node|utils_node" | grep -v grep`
	while [ -z "$IS_EXIST"  ]
	do
			sleep 1
			IS_EXIST=`ps -ef | grep -E "collect_node|utils_node" | grep -v grep`
	done
done

PID=`ps -ef | grep -E "middleware_node|collect_node|planning_node|slam_node|log_recorder" | grep -v grep | awk '{print $2}'`
for item  in $PID
do
	kill -s 9 $item
	echo "kill -s -9 $item"
done

#./log_recorder > /dev/null &

#${node_path}devel/lib/collect_node/collect_node &
#${node_path}devel/lib/middleware_node/middleware_node > /dev/null &
#${node_path}devel/lib/planning_node/planning_node > /dev/null &
${node_path}devel/lib/slam_node/slam_node
#${node_path}devel/lib/utils_node/utils_node &
