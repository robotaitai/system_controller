# PolicyState.msg - Policy state and mode control message
std_msgs/Header header

# Operating modes
uint8 TELEOP_ONLY = 0
uint8 AUTONOMOUS = 1
uint8 SEMI_AUTONOMOUS = 2
uint8 ADAS_ASSISTED = 3
uint8 SAFETY_OVERRIDE = 4
uint8 MAINTENANCE = 5

uint8 current_mode           # Current operating mode
uint8 requested_mode         # Requested operating mode
bool mode_transition_active  # Mode transition in progress

# Policy flags
bool teleop_allowed          # Teleoperation commands allowed
bool autonomy_allowed        # Autonomous commands allowed
bool safety_override_active  # Safety override is active
bool emergency_stop_active   # Emergency stop is active

# Input source priorities (0-100, higher = more priority)
uint8 teleop_priority        # Teleoperation input priority
uint8 autonomy_priority      # Autonomy input priority
uint8 safety_priority        # Safety system priority
uint8 mission_priority       # Mission system priority

# Safety conditions
bool gps_valid              # GPS signal is valid
bool sensor_health_ok       # All sensors are healthy
bool vehicle_health_ok      # Vehicle systems are healthy
bool implement_health_ok    # Implement systems are healthy
float64 safety_zone_distance # Distance to safety zone boundary (meters)

# Mission/implement specific policies
bool spray_allowed          # Spraying operations allowed
bool mowing_allowed         # Mowing operations allowed
bool seeding_allowed        # Seeding operations allowed
string[] restricted_zones   # List of restricted zone IDs
string[] active_constraints # Active policy constraints

# Policy metadata
string policy_id            # Active policy identifier
string policy_version       # Policy version
builtin_interfaces/Time last_update  # Last policy update time
string[] policy_violations  # Current policy violations 