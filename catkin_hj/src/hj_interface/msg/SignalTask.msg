int32 task_type    #0-快速任务 1-计时任务 2-周期任务 3-手动任务
int32 operate      #0-增加任务 1-覆盖任务
string overrideID  #表示要覆盖的任务的名称
string name_id
string start_time
time start_time_stamp
int32 clean_time   # minutes
CleanAreas[] clean_areas
int32 clean_mode   # 1-变频 2-标准