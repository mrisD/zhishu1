# Structured result for commands accepted by zhishu_robot_commander_cpp.
# error_code values intentionally match zhishu_robot_commander_cpp::ErrorCode.
int32 SUCCESS=0
int32 INVALID_ARGUMENT=1
int32 TARGET_REJECTED=2
int32 PLANNING_FAILED=3
int32 EXECUTION_FAILED=4
int32 BUSY=5
int32 CANCELLED=6
int32 TIMEOUT=7
int32 NOT_READY=8
int32 INTERNAL_ERROR=9

builtin_interfaces/Time stamp
string command
bool success
int32 error_code
string error_name
int32 native_code
string message
