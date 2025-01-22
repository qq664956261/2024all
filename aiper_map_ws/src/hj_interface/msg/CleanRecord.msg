## 清洁日志消息
float32 clean_speed             # 清洁速度 单位：m/s
float32 surface_clean_area      # 清洁水面面积 单位：m2
float32 bottom_clean_area       # 清洁池底面积
float32 wall_clean_area         # 清洁池壁面积
float32 water_line_clean_area   # 清洁水线面积
float32 pool_area               # 泳池面积
float32 pool_volume             # 泳池体积
float32 pool_depth              # 泳池最大深度
float32 pool_little_depth       # 泳池最小深度
int32   pool_shape              # 泳池形状
string  map_line_file_path      # 地图轨迹文件路径
int32   avoidObstaclesCount     # 避障次数
int32   getOutCount             # 脱困次数
int32   multiple_bottom         # 多层池底 1-单层 2-多层

#### 以下用于埋点的消息
## 清洁事件消息- 还有一部分放在清洁日志消息中
int32 start_turbidity          # 起始浊度
int32 end_turbidity            # 终止浊度
int32 build_map_faile_reson    # 地图构建失败原因
int32 build_map_fail_count     # 地图构建失败次数
int32 basket_block_time        # 滤篮开始堵塞时间
float32 phy_area               # 物理面积
int32 charge_position          # 充电点位次数
int32 in_water_positon         # 入水点位次数
int32 fall_count               # 跌落次数
int32 loop                     # 回环误差值
int32 density                  # 水密度值
int32 in_water_signal          # 入水点信号强度
int32 in_water_level           # 入水点等级
int32 charge_signal            # 充电信号强度
int32 charge_level             # 充电点等级
#磁力计信息-半小时1次，单任务6次

## 清洁不同区域共同信息
int32 mode                     # 清扫模式   1-水面模式 2-池底 3-池壁 4-水线
float32 clean_area             # 清扫面积
int32[] trapped_reason         # 被困触发条件
int32 trapped_count            # 被困次数
int32 task_status              # 任务状态 0-失败 1-成功
int32 taskFailReason           # 任务失败原因
float32 mapped_area            # 建图面积
int32 untrapped_status         # 脱困状态  0-未脱困 1-脱困
int32 untrapped_count          # 脱困次数

## 水面清洁信息
float32 suface_border_max      # 延边最大距离
float32 suface_border_min      # 延边最小距离
float32 suface_border_avg      # 延边平均距离
float32 suface_border_std      # 延边方差
int32 ject_count               # 水面喷口角度变化的次数（水面）
int32 suface_init_time         # 定位初始化时长

## 池底清洁信息
int32 bottom_avoidance_count   # 避障次数
float32 miss_area              # 单边弓任务漏扫区域面积（池底）
int32 relocation_status        # 重定位状态 0-失败 1-成功
int32 relocation_fail_reason   # 重定位失败原因（池底）
float32 bottom_border_max      # 延边最大距离
float32 bottom_border_min      # 延边最小距离
float32 bottom_border_avg      # 延边平均距离
float32 bottom_border_std      # 延边方差
int32 bottom_border_start_time   # 延边开始时间
int32 bottom_border_end_time     # 延边结束时间
int32 bottom_border_fail_time    # 延边失败时间
int32 bottom_border_status       # 延边状态 0-失败 1-成功
int32 bottom_init_time         # 定位初始化时长
int32[] pitch                  # 俯仰角度数（池底）-单任务5次
int32 under_view               # 下视触发次数（池底）
int32 relocation_start_time    # 重定位开始时间
int32 relocation_end_time      # 重定位结束时间

## 池壁清洁信息
int32 wall_avoidance_count     # 避障次数
# yaw角度，yaw角度占比值
int32[] wall_move              # 池壁侧移触发原因（池壁）
int32 wall_move_count          # 池壁侧移次数（池壁）

## 水线清洁信息
int32 line_avoidance_count     # 避障次数

## 召回
int32 recall_status            # 召回状态 0-失败 1-成功

## 回充
int32 compensate_reason        # 回充原因 0-低电量 1-任务结束 2-召回
int32 compensate_retry         # 回充对桩尝试次数
int32 compensate_status        # 回充上电状态 0-失败 1-成功
