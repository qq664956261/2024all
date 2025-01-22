int32 task_type         #0-快速任务 1-计时任务 2-周期任务 3-手动任务
int32 task_mode         #1-间隔天数周期任务 2-按周周期任务
int32 cancelNowTask     #是否取消今天的周期的任务
int32 taskExecute       #表示周期任务是否生效
PeriodicWeekdayTask[] weekday_task
PeriodicIntervalTask interval_task