string payload # 大数据已组好的内容
string upload_file #带压缩包上传时，压缩包路径
uint8  kBigdataImmediate = 0 # 立刻发送，当前版本策略必带
uint8  kBigdataPack = 1 #需要带压缩包时(upload_file)，带上这个参数
uint8  kBigdataDelete = 2 # 默认压缩包上传成功不删除，带上这个参数会帮你删除
uint8  type #上述参数或关系，如：kBigdataImmediate|kBigdataPack|kBigdataDelete
