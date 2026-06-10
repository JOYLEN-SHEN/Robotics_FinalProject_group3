# Fleet status message - published by fleet manager at 1Hz
std_msgs/Header header

RobotStatus[] robots             # Status of all robots in the fleet
uint32 active_task_count         # Number of currently active tasks
uint32 pending_task_count        # Number of tasks waiting to be assigned
uint32 completed_task_count      # Total completed tasks since startup
uint32 failed_task_count         # Total failed tasks since startup
float32 fleet_utilization        # Percentage of robots actively working [0.0-1.0]
float32 average_task_completion_time  # Average task duration [seconds]
uint32 conflict_count            # Number of path conflicts resolved since startup
bool all_robots_healthy          # True if no robots in ERROR state
