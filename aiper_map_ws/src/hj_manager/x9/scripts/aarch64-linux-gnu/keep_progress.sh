#!/usr/bin/env bash
# 守护进程，每隔5秒守护一次
RESTART_COUNT=1
CIRCULATE_COUNT=0

# 节点名称
COLLECT_NODE=collect_node
SLAM_NODE=slam_node
UTILS_NODE=utils_node
PLANNING_NODE=planning_node
MIDDLEWARE_NODE=middleware_node
NODE_LIST="$COLLECT_NODE $SLAM_NODE $UTILS_NODE $PLANNING_NODE $MIDDLEWARE_NODE"

NODE_MONITOR_JSON="/userdata/hj/log/node_monitor.json"
LOG_ERR0="/userdata/hj/log/logging/log_err0"
LOG_ERR1="/userdata/hj/log/logging/log_err1"
LOG_ERR2="/userdata/hj/log/logging/log_err2"
log_err_file=$LOG_ERR0

# 获取最新日志文件
function get_lastest_file() {
  if [ -f "$LOG_ERR0" ]; then
    log_err0_CurrTimestamp=`date -r "$LOG_ERR0" +%s`
  else
    log_err0_CurrTimestamp=0
  fi

  if [ -f "$LOG_ERR1" ]; then
    log_err1_CurrTimestamp=`date -r "$LOG_ERR1" +%s`
  else
    log_err1_CurrTimestamp=0
  fi

  if [ -f "$LOG_ERR2" ]; then
    log_err2_CurrTimestamp=`date -d "$LOG_ERR2" +%s`
  else
    log_err2_CurrTimestamp=0
  fi
  
  if [ $log_err0_CurrTimestamp -gt $log_err1_CurrTimestamp -a $log_err0_CurrTimestamp -gt $log_err2_CurrTimestamp ]; then
    log_err_file=$LOG_ERR0
  elif [ $log_err1_CurrTimestamp -gt $log_err0_CurrTimestamp -a $log_err1_CurrTimestamp -gt $log_err2_CurrTimestamp ]; then
    log_err_file=$LOG_ERR1
  else
    log_err_file=$LOG_ERR2
  fi
}

# 节点监听，挂掉重启
function restart_func()
{
  local node_exist=1
  for node in $NODE_LIST
  do
    PID=`pidof $node`
    if [ -z "$PID" ]; then
      get_lastest_file
      local timestamp_s=`date +%s`
      local timestamp_nano=`date +%N`
      local timestamp=${timestamp_s}.${timestamp_nano:0:3}
      local error_str='{"node": "'$node'","timestamp":'$timestamp',"restart_count":'$RESTART_COUNT'}'
      echo $error_str >> $NODE_MONITOR_JSON
      echo $error_str >> $log_err_file
      node_exist=0
    fi
  done
  local manage_count=`ps -ef | grep -E "hj_manager.sh" | grep -v grep | wc -l`
  if [ $node_exist -eq 0 -a $manage_count -eq 0 ]; then
    RESTART_COUNT=$((RESTART_COUNT+1))
    /data/hj/bin/hj_manager.sh "open" > /dev/null
    CIRCULATE_COUNT=0
  fi
}

# 主函数
while true
do
  sleep 10    #守护进程运行间隔
  
  if [ $RESTART_COUNT -gt 3 ]; then #重启次数超过3次，不再重启
    get_lastest_file
    echo "hj_manager restart too many times, restart_count: $RESTART_COUNT" >> $log_err_file
    exit 1
  else
    hj_manager_count=`ps -ef | grep -E "hj_manager.sh" | grep -v grep | wc -l`
    if [ "$hj_manager_count" -eq 0 ]; then
      restart_func
    fi
  fi
  CIRCULATE_COUNT=$((CIRCULATE_COUNT+1))
  if [ $CIRCULATE_COUNT -gt 12 ]; then #运行120秒，重置重启次数
    RESTART_COUNT=1
    CIRCULATE_COUNT=0
  fi

done
