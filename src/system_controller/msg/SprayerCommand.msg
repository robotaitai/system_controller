# SprayerCommand.msg - Sprayer-specific command message
std_msgs/Header header

# Basic implement identification and control
string implement_id           # Unique implement identifier  
bool activate                 # Activate/deactivate sprayer
bool raise                    # Raise sprayer (true) or lower (false)
float64 height_position       # Specific height position (meters)

# Sprayer specific commands
bool valve_open              # Open/close valve for sprayers
float64 spray_rate           # Spray rate (L/min or L/ha)
float64 spray_pressure       # Spray pressure (bar)
string chemical_type         # Type of chemical being sprayed
float64 application_rate     # Application rate (L/ha)
float64 boom_width          # Boom width in meters
bool[] nozzle_enable        # Enable/disable individual nozzles
uint8 spray_pattern         # 0=flat fan, 1=hollow cone, 2=full cone

# Advanced sprayer features
float64 tank_level          # Current tank level percentage (0-100)
bool auto_section_control   # Enable automatic section control
float64 overlap_reduction   # Overlap reduction percentage (0-100)
bool drift_reduction        # Enable drift reduction mode

# Environmental considerations
float64 wind_compensation   # Wind compensation factor
bool weather_monitoring     # Enable weather-based control
float64 temperature_limit   # Maximum operating temperature (C)
float64 wind_speed_limit    # Maximum wind speed (m/s)

# Safety and status
bool emergency_stop          # Emergency stop for sprayer
uint8 safety_mode           # 0=normal, 1=safe, 2=maintenance
string[] active_zones       # Active sprayer zones/sections

# Command metadata  
string command_id           # Unique command identifier
builtin_interfaces/Duration timeout  # Command timeout
float64 priority            # Command priority (0-100) 