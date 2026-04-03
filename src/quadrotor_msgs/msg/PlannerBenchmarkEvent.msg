Header header
int32 drone_id
uint32 trajectory_id
uint8 previous_state
uint8 current_state
uint8 event_type
uint8 trigger_reason
bool success
float64 goal_distance
string caller

uint8 STATE_INIT=0
uint8 STATE_WAIT_TARGET=1
uint8 STATE_GEN_NEW_TRAJ=2
uint8 STATE_REPLAN_TRAJ=3
uint8 STATE_EXEC_TRAJ=4
uint8 STATE_EMERGENCY_STOP=5
uint8 STATE_SEQUENTIAL_START=6

uint8 EVENT_STATE_TRANSITION=1
uint8 EVENT_PLAN_RESULT=2
uint8 EVENT_GOAL_REACHED=3
uint8 EVENT_TRIGGER_RECEIVED=4