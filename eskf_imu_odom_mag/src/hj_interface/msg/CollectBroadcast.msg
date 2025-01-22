uint8 action    # action type: see enum SYS_ACTION_* [references SysAction.msg]
uint8 function  # function type
uint8 ack  # 1: action oK, 0: action failed
uint8 SYS_ACTION_SHUTDOWN = 0
uint8 SYS_ACTION_RESET = 1      # reset from remote control
uint8 SYS_ACTION_RESTORE_FACTORY = 2    # restore factory from key
uint8 SYS_ACTION_SLEEP = 3

