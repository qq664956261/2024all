#!/usr/bin/env bash
ABSOLUTE_PATH="/data/hj"

echo $#
if [ $# == 1 ]; then
  ABSOLUTE_PATH=$1
  echo ${ABSOLUTE_PATH}
fi

source /opt/ros/melodic/setup.bash
export LD_LIBRARY_PATH=/${ABSOLUTE_PATH}/lib:${LD_LIBRARY_PATH}:/${ABSOLUTE_PATH}/../glog

export HJ_LOG_CONFIG_PATH="${ABSOLUTE_PATH}/config/hj_log.config"
export LOG_RECORDER_NEW_PATH="${ABSOLUTE_PATH}/config/log_recorder.json"
#export HJ_ALL_LOG_CLOSE=close
#export HJ_LOG_CLOSE_collect_node=close
mkdir /tmp/logging
rm /tmp/logging/log_err

while true; do
    COUNT=$(ps -A | grep roscore | wc -l)
    if [ "$COUNT" != "0" ]; then
        break
    fi
    roscore &
    sleep 1 # wait roscore running
done

PID=`ps -ef | grep -E "record" | grep -v grep | awk '{print $2}'`
for item  in $PID
do
        kill -2 $item
        echo "kill -2 $item"
done

PID=`ps -ef | grep -E "monitor.sh|middleware_node|planning_node|slam_node|utils_node|log_recorder" | grep -v grep | awk '{print $2}'`
for item  in $PID
do
	kill -s 9 $item
	echo "kill -s 9 $item"
done

PID=`ps -ef | grep -E "collect_node" | grep -v grep | awk '{print $2}'`
for item  in $PID
do
	kill -s 2 $item
	echo "kill -s 2 $item"
	IS_EXIST=`ps -ef | grep -E "collect_node" | grep -v grep`
	while [ -z "$IS_EXIST"  ]
	do
			echo "steal exit"
			sleep 1
	done
done


retry_cnt=0;
while [ __1 == __1 ]
do
    if [ $retry_cnt -gt 10 ]; then
        echo "rosparam set fail, exit"
        exit -1
    fi
    param_hj_so=$(rosparam set /hj_so_path "${ABSOLUTE_PATH}/lib" 2>&1)
    param_hj_config=$(rosparam set /hj_config_path "${ABSOLUTE_PATH}/config" 2>&1)

    if [[ "$param_hj_so" == *"ERROR"* || "$param_hj_config" == *"ERROR"* ]]; then
        echo "rosparam set error, retry"
        ((retry_cnt++))
        sleep 1
    else
        echo "rosparam set success"
        break
    fi
done

retry_cnt=0;
while [ __1 == __1 ]
do
    if [ $retry_cnt -gt 10 ]; then
        echo "rosparam get fail, exit"
        exit -1
    fi
    param_hj_so=$(rosparam get /hj_so_path 2>&1)
    param_hj_config=$(rosparam get /hj_config_path 2>&1)

    if [[ "$param_hj_so" == *"ERROR"* || "$param_hj_config" == *"ERROR"* ]]; then
        echo "rosparam get error, retry"
        ((retry_cnt++))
        sleep 1
    else
        echo "rosparam get success"
        break
    fi
done


#rosparam set /hj_so_path "${ABSOLUTE_PATH}/lib"
#rosparam set /hj_config_path "${ABSOLUTE_PATH}/config"
#./log_recorder > /dev/null &

${ABSOLUTE_PATH}/bin/collect_node > /dev/null &
${ABSOLUTE_PATH}/bin/middleware_node > /dev/null &
${ABSOLUTE_PATH}/bin/planning_node > /dev/null &
${ABSOLUTE_PATH}/bin/slam_node > /dev/null &
${ABSOLUTE_PATH}/bin/utils_node > /dev/null &

#sleep 2
#${ABSOLUTE_PATH}/bin/monitor.sh  &
