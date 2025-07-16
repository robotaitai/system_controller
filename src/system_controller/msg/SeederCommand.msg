# SeederCommand.msg - Seeder-specific command message
std_msgs/Header header

# Basic implement identification and control
string implement_id           # Unique implement identifier
bool activate                 # Activate/deactivate seeder
bool raise                    # Raise seeder (true) or lower (false)
float64 height_position       # Specific height position (meters)

# Seeder specific commands
float64 seed_rate            # Seeding rate (seeds/meter or kg/ha)
float64 planting_depth       # Planting depth in meters
bool metering_enable         # Enable seed metering
string seed_type            # Type of seed being planted
float64 row_spacing         # Row spacing in meters

# Advanced seeding features
uint8 metering_system       # 0=mechanical, 1=pneumatic, 2=electric
float64 down_pressure       # Down pressure for seed placement (N)
bool variable_rate_seeding  # Enable variable rate seeding
float64 emergence_rate      # Expected emergence rate percentage
bool population_monitor     # Enable population monitoring

# Soil and conditions
float64 soil_moisture       # Required soil moisture percentage
float64 soil_temperature    # Required soil temperature (C)
bool tillage_required       # Tillage required before seeding
uint8 seed_bed_quality      # Required seed bed quality (0-10)

# Fertilizer integration
bool fertilizer_application # Enable fertilizer application
float64 fertilizer_rate     # Fertilizer rate (kg/ha)
string fertilizer_type      # Type of fertilizer
float64 fertilizer_depth    # Fertilizer placement depth (meters)

# Precision features
bool gps_guidance          # Enable GPS-guided precision seeding
float64 section_control    # Section control width (meters)
bool overlap_reduction     # Enable overlap reduction
bool boundary_tracking     # Enable boundary tracking

# Safety and status
bool emergency_stop          # Emergency stop for seeder
uint8 safety_mode           # 0=normal, 1=safe, 2=maintenance
string[] active_zones       # Active seeder zones/sections

# Command metadata
string command_id           # Unique command identifier
builtin_interfaces/Duration timeout  # Command timeout
float64 priority            # Command priority (0-100) 