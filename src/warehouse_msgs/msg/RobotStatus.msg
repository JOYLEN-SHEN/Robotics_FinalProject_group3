# Robot status message - published by each robot and monitored by fleet manager
std_msgs/Header header

string robot_id                  # Unique robot identifier (e.g., "robot_1")

# Robot state enum
uint8 STATE_IDLE = 0
uint8 STATE_NAVIGATING = 1
uint8 STATE_LOADING = 2
uint8 STATE_UNLOADING = 3
uint8 STATE_CHARGING = 4
uint8 STATE_ERROR = 5
uint8 STATE_WAITING = 6          # Waiting due to conflict resolution

uint8 state                      # Current robot state

float32 battery_level            # Battery percentage [0.0 - 100.0]
string current_task_id           # ID of currently assigned task (empty if IDLE)
geometry_msgs/Pose current_pose  # Current robot pose in map frame
float32 linear_velocity          # Current linear velocity [m/s]
float32 angular_velocity         # Current angular velocity [rad/s]
float32 task_progress            # Progress of current task [0.0 - 1.0]
string error_message             # Error description if in ERROR state
builtin_interfaces/Time last_heartbeat  # Last heartbeat timestamp
