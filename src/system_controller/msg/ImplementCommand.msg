# ImplementCommand.msg - Standard implement command message for mission control
std_msgs/Header header

# Implement identification
string implement_type          # Type of implement (sprayer, mower, seeder, etc.)
string implement_id           # Unique implement identifier

# Basic implement control
bool activate                 # Activate/deactivate implement
bool raise                    # Raise implement (true) or lower (false)
float64 height_position       # Specific height position (meters)

# Sprayer specific commands
bool valve_open              # Open/close valve for sprayers
float64 spray_rate           # Spray rate (L/min or similar)
float64 spray_pressure       # Spray pressure (bar)
string chemical_type         # Type of chemical being sprayed

# Mower specific commands  
float64 cutting_height       # Cutting height in meters
float64 blade_speed          # Blade rotation speed (RPM)
bool side_discharge          # Enable side discharge

# Seeder specific commands
float64 seed_rate            # Seeding rate (seeds/meter or kg/ha)
float64 planting_depth       # Planting depth in meters
bool metering_enable         # Enable seed metering

# Safety and status
bool emergency_stop          # Emergency stop for implement
uint8 safety_mode           # 0=normal, 1=safe, 2=maintenance
string[] active_zones       # Active implement zones/sections

# Command metadata
string command_id           # Unique command identifier
builtin_interfaces/Duration timeout  # Command timeout
float64 priority            # Command priority (0-100) 