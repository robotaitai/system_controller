# MowerCommand.msg - Mower-specific command message
std_msgs/Header header

# Basic implement identification and control
string implement_id           # Unique implement identifier
bool activate                 # Activate/deactivate mower
bool raise                    # Raise mower (true) or lower (false)
float64 height_position       # Specific height position (meters)

# Mower specific commands
float64 cutting_height       # Cutting height in meters
float64 blade_speed          # Blade rotation speed (RPM)
bool side_discharge          # Enable side discharge
uint8 cutting_pattern       # 0=stripe, 1=spiral, 2=random
float64 overlap_percentage   # Cutting overlap percentage

# Advanced mower features
bool mulching_mode          # Enable mulching mode
bool grass_collection       # Enable grass collection
float64 ground_speed        # Recommended ground speed (m/s)
bool auto_height_control    # Enable automatic height adjustment
float64 blade_engagement    # Blade engagement force (0-100%)

# Terrain adaptation
bool slope_compensation     # Enable slope compensation
float64 max_slope_angle     # Maximum safe slope angle (degrees)
bool anti_scalp_wheels      # Enable anti-scalp wheels
bool deck_float            # Enable deck floating

# Quality control
uint8 cut_quality_mode      # 0=fast, 1=normal, 2=fine
bool striping_kit          # Enable striping kit
float64 blade_sharpness     # Blade condition indicator (0-100%)

# Safety and status
bool emergency_stop          # Emergency stop for mower
uint8 safety_mode           # 0=normal, 1=safe, 2=maintenance
string[] active_zones       # Active mower zones/sections
bool operator_presence      # Operator presence required

# Command metadata
string command_id           # Unique command identifier
builtin_interfaces/Duration timeout  # Command timeout
float64 priority            # Command priority (0-100) 