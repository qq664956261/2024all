int32 cmd           # 1-水面 2-池底 3-池壁 
                    # 11-建图 12-定位  
                    # 21-召回 22-回充 
                    # 31-导航状态查询 32-清洁模式  
                    # 101-pause, 102-resume, 103-stop, 104-fail
int32 errcode       # error code - 任务失败填充这个错误码
string msg          # message - 有错误码可以不填这个msg
