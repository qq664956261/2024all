#!/usr/bin/env bash
PID=`ps -ef | grep -E "keep_progress.sh|middleware_node|planning_node|slam_node|utils_node|collect_node|log_recorder|rosctl_ser" | grep -v grep | awk '{print $1}'`
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
