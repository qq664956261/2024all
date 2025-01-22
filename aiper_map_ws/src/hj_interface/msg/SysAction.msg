uint8 action    # system action type: see enum SYS_ACTION_*
uint8 res       # result: 0 means OK
uint8 from      # source node : see enum COMM_NODE_*
uint8 SYS_ACTION_SHUTDOWN = 0
uint8 SYS_ACTION_RESET = 1      # reset from remote control
uint8 SYS_ACTION_RESTORE_FACTORY = 2    # restore factory from key
uint8 SYS_ACTION_SLEEP = 3
uint8 COMM_NODE_COLLECT = 1
uint8 COMM_NODE_MIDDLEWARE = 2
uint8 COMM_NODE_PLANNING = 3
uint8 COMM_NODE_SLAM = 4
uint8 COMM_NODE_UTILS = 5
uint8 COMM_NODE_OS = 6