# ImplementCommand.msg - Base implement command message for generic control
std_msgs/Header header

# Implement identification
string implement_type          # Type of implement (sprayer, mower, seeder, etc.)
string implement_id           # Unique implement identifier

# Basic implement control
bool activate                 # Activate/deactivate implement
bool raise                    # Raise implement (true) or lower (false)
float64 height_position       # Specific height position (meters)

# Safety and status
bool emergency_stop          # Emergency stop for implement
uint8 safety_mode           # 0=normal, 1=safe, 2=maintenance
string[] active_zones       # Active implement zones/sections

# Command metadata
string command_id           # Unique command identifier
builtin_interfaces/Duration timeout  # Command timeout
float64 priority            # Command priority (0-100) 