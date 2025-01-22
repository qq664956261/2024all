string      todo   # 当前操作为OTA升级 UpdateOTA: ota升级 Reboot: 机器重启
int32       stage  # 0 下载 1 升级
int32       module # 1 mcu 2 power 3 core 注: stage=1时 关注此字段
subPackData data   # 固件包子包的地址和md5