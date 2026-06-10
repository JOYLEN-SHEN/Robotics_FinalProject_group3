# Task request message - sent to fleet manager to request a new task
std_msgs/Header header

string task_id                   # Unique task identifier (UUID)

# Task type enum
uint8 TASK_PICKUP_DELIVERY = 0   # Pickup from one location, deliver to another
uint8 TASK_CHARGING = 1          # Go to charging station
uint8 TASK_IDLE_RETURN = 2       # Return to home position

uint8 task_type

geometry_msgs/Pose pickup_location    # Where to pick up the item
geometry_msgs/Pose dropoff_location   # Where to drop off the item
string pickup_zone                    # Human-readable zone name (e.g., "shelf_A3")
string dropoff_zone                   # Human-readable zone name (e.g., "loading_dock_1")

# Priority enum
uint8 PRIORITY_LOW = 0
uint8 PRIORITY_NORMAL = 1
uint8 PRIORITY_HIGH = 2
uint8 PRIORITY_CRITICAL = 3

uint8 priority
float32 estimated_weight         # Estimated weight of payload [kg]
builtin_interfaces/Time deadline # Deadline for task completion (zero = no deadline)
string requester_id              # ID of entity requesting the task
