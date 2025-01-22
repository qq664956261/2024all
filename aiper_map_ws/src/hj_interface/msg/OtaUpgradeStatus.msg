string   todo      # 当前操作为OTA升级 UpdateOTA：ota升级
int32    stage     # 0 下载 1 升级
float32  progress  # 下载或者升级进度
int32    module    # 1 mcu 2 power 3 core 注: stage=1时 关注此字段
int32    ret       # 升级结果 0 成功；-1 失败
string   msg       # 升级失败时，填充失败原因