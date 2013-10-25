int32 seq
float64 percent_complete

# status needs to be within [0..9]
int32 status
int32 IDLE=0
int32 STARTED=1
int32 PREEMPTED=2
int32 SWAPPED=3
int32 FINISHED=4
int32 FAILED=5
int32 STOPPED=6
int32 CONTINUED=7

time start_time
time end_time
