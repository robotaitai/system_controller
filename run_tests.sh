#!/bin/bash

# ROS 2 System Controller Test Runner
# This script runs all tests locally (inside container or with native ROS 2)

set -e  # Exit on error

echo "🧪 ROS 2 System Controller - Comprehensive Test Suite"
echo "====================================================="
echo ""

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Function to print colored output
print_status() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

print_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# Check if we're in ROS environment
check_ros_environment() {
    if [ -z "$ROS_DISTRO" ]; then
        print_error "ROS 2 environment not detected!"
        echo "Please source your ROS 2 setup:"
        echo "  source /opt/ros/humble/setup.bash"
        echo "  source install/setup.bash"
        exit 1
    fi
    print_success "ROS 2 environment detected: $ROS_DISTRO"
}

# Function to run a test and capture results
run_test() {
    local test_name="$1"
    local test_command="$2"
    local test_type="$3"
    
    print_status "Running $test_type: $test_name"
    
    if eval "$test_command"; then
        print_success "$test_name passed"
        return 0
    else
        print_error "$test_name failed"
        return 1
    fi
}

# Test counters
TOTAL_TESTS=0
PASSED_TESTS=0
FAILED_TESTS=0

increment_test() {
    TOTAL_TESTS=$((TOTAL_TESTS + 1))
    if [ $? -eq 0 ]; then
        PASSED_TESTS=$((PASSED_TESTS + 1))
    else
        FAILED_TESTS=$((FAILED_TESTS + 1))
    fi
}

# Main test execution
main() {
    echo "Starting test execution at $(date)"
    echo ""
    
    # Check environment
    check_ros_environment
    
    # Test 1: Core class functionality (standalone)
    echo "🧩 Testing Core Classes"
    echo "-----------------------"
    if [ -f "./test_core_classes" ]; then
        run_test "Core Classes" "./test_core_classes" "Unit Test"
        increment_test
    else
        print_warning "Core classes test binary not found. Compiling..."
        if g++ -std=c++17 -o test_core_classes test_core_classes.cpp -lgtest -lgtest_main -pthread 2>/dev/null; then
            run_test "Core Classes" "./test_core_classes" "Unit Test"
            increment_test
        else
            print_warning "Google Test not available for standalone compilation"
        fi
    fi
    echo ""
    
    # Test 2: Local simulation
    echo "🚀 Testing Local Simulation"
    echo "---------------------------"
    if [ -f "./run_local_simulation" ]; then
        run_test "Local Simulation" "timeout 30s ./run_local_simulation" "Integration Test"
        increment_test
    else
        print_warning "Local simulation binary not found. Compiling..."
        if g++ -std=c++17 -pthread -o run_local_simulation run_local_simulation.cpp; then
            run_test "Local Simulation" "timeout 30s ./run_local_simulation" "Integration Test"
            increment_test
        else
            print_error "Failed to compile local simulation"
            increment_test
        fi
    fi
    echo ""
    
    # Test 3: ROS 2 package build
    echo "🔨 Testing ROS 2 Package Build"
    echo "------------------------------"
    if command -v colcon >/dev/null 2>&1; then
        run_test "Package Build" "colcon build --packages-select system_controller" "Build Test"
        increment_test
        
        # Source the built package
        if [ -f "install/setup.bash" ]; then
            source install/setup.bash
            print_success "Package built and sourced successfully"
        fi
    else
        print_warning "colcon not available, skipping ROS 2 build test"
    fi
    echo ""
    
    # Test 4: ROS 2 unit tests (if built)
    echo "🔬 Testing ROS 2 Unit Tests"
    echo "---------------------------"
    if [ -f "build/system_controller/test_core_classes" ]; then
        run_test "ROS 2 Core Tests" "./build/system_controller/test_core_classes" "ROS 2 Unit Test"
        increment_test
    else
        print_warning "ROS 2 test binaries not found. Run 'colcon build' first."
    fi
    
    if [ -f "build/system_controller/test_regression" ]; then
        run_test "ROS 2 Regression Tests" "./build/system_controller/test_regression" "ROS 2 Regression Test"
        increment_test
    fi
    echo ""
    
    # Test 5: Node startup tests
    echo "⚙️  Testing Node Startup"
    echo "------------------------"
    if command -v ros2 >/dev/null 2>&1 && [ -f "install/setup.bash" ]; then
        # Test if nodes can be imported/started
        run_test "System Manager Node" "timeout 5s ros2 run system_controller system_manager_node --help" "Node Test"
        increment_test
        
        run_test "Command Arbiter Node" "timeout 5s ros2 run system_controller command_input_arbiter_node --help" "Node Test" 
        increment_test
    else
        print_warning "ROS 2 not available or package not built, skipping node tests"
    fi
    echo ""
    
    # Test 6: Message/Service definitions
    echo "📡 Testing Message/Service Definitions"
    echo "--------------------------------------"
    if command -v ros2 >/dev/null 2>&1 && [ -f "install/setup.bash" ]; then
        run_test "System Command Message" "ros2 interface show system_controller/msg/SystemCommand" "Interface Test"
        increment_test
        
        run_test "Get Status Service" "ros2 interface show system_controller/srv/GetStatus" "Interface Test"
        increment_test
    else
        print_warning "ROS 2 not available, skipping interface tests"
    fi
    echo ""
    
    # Test 7: Launch file validation
    echo "🚁 Testing Launch Files"
    echo "-----------------------"
    if command -v ros2 >/dev/null 2>&1 && [ -f "install/setup.bash" ]; then
        if [ -f "src/system_controller/launch/system_controller.launch.py" ]; then
            run_test "Main Launch File" "python3 -m py_compile src/system_controller/launch/system_controller.launch.py" "Launch Test"
            increment_test
        fi
        
        if [ -f "src/system_controller/launch/demo.launch.py" ]; then
            run_test "Demo Launch File" "python3 -m py_compile src/system_controller/launch/demo.launch.py" "Launch Test"
            increment_test
        fi
    else
        print_warning "Python or ROS 2 not available, skipping launch file tests"
    fi
    echo ""
    
    # Performance test (basic)
    echo "⚡ Testing Performance (Basic)"
    echo "-----------------------------"
    if [ -f "./test_core_classes" ]; then
        print_status "Running performance measurement..."
        start_time=$(date +%s%N)
        ./test_core_classes > /dev/null 2>&1
        end_time=$(date +%s%N)
        duration=$((($end_time - $start_time) / 1000000))  # Convert to milliseconds
        
        if [ $duration -lt 5000 ]; then  # Less than 5 seconds
            print_success "Performance test passed (${duration}ms)"
        else
            print_warning "Performance test slow (${duration}ms)"
        fi
        increment_test
    fi
    echo ""
    
    # Final report
    echo "📊 Test Summary"
    echo "==============="
    echo "Total tests run: $TOTAL_TESTS"
    echo "Passed: $PASSED_TESTS"
    echo "Failed: $FAILED_TESTS"
    echo ""
    
    if [ $FAILED_TESTS -eq 0 ]; then
        print_success "🎉 All tests passed!"
        echo ""
        echo "Your ROS 2 System Controller is ready for deployment!"
        echo ""
        echo "Next steps:"
        echo "  1. Push to GitHub to trigger CI/CD"
        echo "  2. Deploy to GitHub Codespaces for full ROS 2 testing"
        echo "  3. Run full system tests with:"
        echo "     ros2 launch system_controller system_controller.launch.py"
        exit 0
    else
        print_error "❌ Some tests failed!"
        echo ""
        echo "Please check the failed tests above and fix any issues."
        echo "Common solutions:"
        echo "  1. Ensure ROS 2 environment is properly sourced"
        echo "  2. Run 'colcon build' to build the package"
        echo "  3. Check dependencies with 'rosdep install --from-paths src --ignore-src -r -y'"
        exit 1
    fi
}

# Handle script arguments
case "${1:-}" in
    --help|-h)
        echo "Usage: $0 [OPTIONS]"
        echo ""
        echo "Options:"
        echo "  --help, -h     Show this help message"
        echo "  --quick, -q    Run only quick tests (no simulation)"
        echo "  --full, -f     Run all tests including integration tests"
        echo ""
        echo "Examples:"
        echo "  $0              # Run standard test suite"
        echo "  $0 --quick      # Run only unit tests"
        echo "  $0 --full       # Run comprehensive test suite"
        exit 0
        ;;
    --quick|-q)
        print_status "Running quick test suite..."
        # Set flags to skip long-running tests
        QUICK_MODE=1
        ;;
    --full|-f)
        print_status "Running comprehensive test suite..."
        FULL_MODE=1
        ;;
    "")
        print_status "Running standard test suite..."
        ;;
    *)
        print_error "Unknown option: $1"
        echo "Use --help for usage information"
        exit 1
        ;;
esac

# Run main function
main 