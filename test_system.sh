#!/bin/bash

# ROS 2 System Controller Test Script
# This script helps verify that the system is working correctly

echo "=== ROS 2 System Controller Test Script ==="
echo ""

# Check if we're in a ROS 2 environment
if [ -z "$ROS_DISTRO" ]; then
    echo "❌ ROS 2 environment not detected. Please source setup.bash first:"
    echo "   source /opt/ros/humble/setup.bash"
    echo "   source install/setup.bash"
    exit 1
fi

echo "✅ ROS 2 environment detected: $ROS_DISTRO"
echo ""

# Function to check if a node is running
check_node() {
    local node_name="$1"
    if ros2 node list | grep -q "$node_name"; then
        echo "✅ Node $node_name is running"
        return 0
    else
        echo "❌ Node $node_name is NOT running"
        return 1
    fi
}

# Function to check if a topic exists
check_topic() {
    local topic_name="$1"
    if ros2 topic list | grep -q "$topic_name"; then
        echo "✅ Topic $topic_name exists"
        return 0
    else
        echo "❌ Topic $topic_name does NOT exist"
        return 1
    fi
}

# Function to send test command and check response
test_command() {
    local topic="$1"
    local message_type="$2"
    local data="$3"
    
    echo "📤 Sending test command to $topic: $data"
    timeout 2s ros2 topic pub --once "$topic" "$message_type" "$data"
    
    if [ $? -eq 0 ]; then
        echo "✅ Command sent successfully"
    else
        echo "❌ Failed to send command"
    fi
}

# Test 1: Check if key nodes are running
echo "=== Test 1: Checking Node Status ==="
check_node "system_manager"
check_node "command_input_arbiter"
check_node "vehicle_adapter_manager"
check_node "telemetry_collector"
check_node "mission_service"
check_node "mission_adapter_manager"
echo ""

# Test 2: Check if key topics exist
echo "=== Test 2: Checking Topic Status ==="
check_topic "/PolicyCommand"
check_topic "/ArbitratedCommand"
check_topic "/VehicleStatus"
check_topic "/MissionStatus"
echo ""

# Test 3: Send test commands
echo "=== Test 3: Testing Command Flow ==="

echo "🔄 Testing teleop command (high priority)..."
test_command "/Teleop/Command" "std_msgs/String" "data: 'test_teleop_command'"
sleep 1

echo "🔄 Testing autonomy command (medium priority)..."
test_command "/Autonomy/Command" "std_msgs/String" "data: 'test_autonomy_command'"
sleep 1

echo "🔄 Testing mission command..."
test_command "/MissionSel/Command" "std_msgs/String" "data: 'start'"
sleep 1

echo ""

# Test 4: Check services
echo "=== Test 4: Testing Services ==="

echo "🔍 Testing vehicle adapter status service..."
timeout 5s ros2 service call /vehicle_adapter_status system_controller/srv/GetStatus "{component_id: 'test'}" 2>/dev/null
if [ $? -eq 0 ]; then
    echo "✅ Vehicle adapter service responded"
else
    echo "❌ Vehicle adapter service did not respond"
fi

echo "🔍 Testing mission service status service..."
timeout 5s ros2 service call /mission_service_status system_controller/srv/GetStatus "{component_id: 'test'}" 2>/dev/null
if [ $? -eq 0 ]; then
    echo "✅ Mission service responded"
else
    echo "❌ Mission service did not respond"
fi

echo ""

# Test 5: Monitor message flow briefly
echo "=== Test 5: Monitoring Message Flow (5 seconds) ==="
echo "📊 Monitoring /PolicyCommand for 5 seconds..."
timeout 5s ros2 topic echo /PolicyCommand --once 2>/dev/null
if [ $? -eq 0 ]; then
    echo "✅ Policy commands are being published"
else
    echo "❌ No policy commands detected"
fi

echo "📊 Monitoring /VehicleStatus for 5 seconds..."
timeout 5s ros2 topic echo /VehicleStatus --once 2>/dev/null
if [ $? -eq 0 ]; then
    echo "✅ Vehicle status is being published"
else
    echo "❌ No vehicle status detected"
fi

echo ""
echo "=== Test Summary ==="
echo "If you see mostly ✅ symbols above, your system is working correctly!"
echo ""
echo "To start monitoring the system manually:"
echo "  ros2 topic list                    # List all topics"
echo "  ros2 node list                     # List all nodes"
echo "  ros2 topic echo /PolicyCommand     # Monitor policy commands"
echo "  ros2 topic echo /ArbitratedCommand # Monitor arbitrated commands"
echo "  ros2 topic echo /VehicleStatus     # Monitor vehicle status"
echo ""
echo "To send manual commands:"
echo "  ros2 topic pub /Teleop/Command std_msgs/String \"data: 'manual_command'\""
echo "  ros2 topic pub /MissionSel/Command std_msgs/String \"data: 'start'\""
echo ""
echo "Happy testing! 🚀" 