# VehicleCommand.msg - Standard vehicle command message for all high-level motion commands
std_msgs/Header header

# Motion control
float64 steering_angle          # Steering angle in radians (-pi to pi)
float64 velocity                # Forward velocity in m/s
float64 acceleration            # Desired acceleration in m/s^2
float64 angular_velocity        # Angular velocity in rad/s (for differential drive)

# Additional control modes
uint8 drive_mode               # 0=manual, 1=autonomous, 2=assisted
bool enable_safety_systems    # Enable/disable safety systems
bool emergency_stop           # Emergency stop flag

# Vehicle specific parameters
float64 max_steering_rate     # Maximum steering rate rad/s
float64 max_acceleration      # Maximum acceleration m/s^2
float64 max_velocity          # Maximum velocity m/s

# Timestamps and IDs
string command_id             # Unique command identifier
builtin_interfaces/Duration timeout  # Command timeout 