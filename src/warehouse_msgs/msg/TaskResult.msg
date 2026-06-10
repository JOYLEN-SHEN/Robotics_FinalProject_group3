# Task result message - published when a task completes or fails
std_msgs/Header header

string task_id                   # ID of the completed/failed task
string robot_id                  # ID of the robot that executed the task
bool success                     # True if task completed successfully
float32 completion_time          # Time taken to complete task [seconds]
float32 distance_traveled        # Total distance traveled for this task [meters]
float32 battery_consumed         # Battery percentage consumed [%]
uint32 replanning_count          # Number of times route was replanned
string failure_reason            # Reason for failure (empty if success)
builtin_interfaces/Time start_time    # When task execution began
builtin_interfaces/Time end_time      # When task completed/failed
