time timestamp 
uint32 front_l 
uint32 front_m
uint32 front_r
uint32 mid
uint32 back
uint8 status #!< 超声波状态,0 means normal,1 means error. 
             #!< 三合一超声波 bit0, 侧边mid超声波bit1 侧边back超声波bit2
uint8 id #!< 超声波id,0 means front,1 means mid,2 means back
