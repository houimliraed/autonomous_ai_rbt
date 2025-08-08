#!/bin/bash

# Pre-Deployment Validation Script for AutonomBot
# Run this before taking the project to the lab

echo "=========================================="
echo "🚀 AUTONOMBOT PRE-DEPLOYMENT VALIDATION"
echo "=========================================="

# Color codes
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

passed=0
failed=0

# Helper functions
check_pass() {
    echo -e "${GREEN}✓ PASS${NC} $1"
    ((passed++))
}

check_fail() {
    echo -e "${RED}✗ FAIL${NC} $1"
    ((failed++))
}

check_warn() {
    echo -e "${YELLOW}⚠ WARN${NC} $1"
}

echo -e "${BLUE}📦 Checking Package Structure...${NC}"

# Check if all packages exist
packages=(
    "autonombot_launch"
    "autonombot_drivers" 
    "autonombot_description"
    "autonombot_firmware"
    "autonombot_localization"
    "autonombot_mapping"
    "autonombot_navigation"
    "autonombot_vision"
)

for package in "${packages[@]}"; do
    if [ -d "src/$package" ]; then
        check_pass "Package $package exists"
    else
        check_fail "Package $package missing"
    fi
done

echo -e "${BLUE}🔧 Checking Launch Files...${NC}"

# Check critical launch files
launch_files=(
    "src/autonombot_launch/launch/real_robot.launch.py"
    "src/autonombot_launch/launch/simulated_robot.launch.py"
    "src/autonombot_firmware/launch/hardware_interface.launch.py"
    "src/autonombot_firmware/launch/jetson_hardware.launch.py"
)

for launch_file in "${launch_files[@]}"; do
    if [ -f "$launch_file" ]; then
        check_pass "Launch file $(basename $launch_file) exists"
    else
        check_fail "Launch file $launch_file missing"
    fi
done

echo -e "${BLUE}📋 Checking Configuration Files...${NC}"

# Check configuration files
config_files=(
    "src/autonombot_launch/config/rplidar_a1.yaml"
    "src/autonombot_drivers/config/autonombot_drivers.yaml"
    "src/autonombot_navigation/config/controller_server.yaml"
    "src/autonombot_navigation/config/advanced_navigation.yaml"
)

for config_file in "${config_files[@]}"; do
    if [ -f "$config_file" ]; then
        check_pass "Config file $(basename $config_file) exists"
    else
        check_fail "Config file $config_file missing"
    fi
done

echo -e "${BLUE}🛠️ Checking Hardware Scripts...${NC}"

# Check hardware setup scripts
if [ -f "src/autonombot_firmware/scripts/jetson_setup.sh" ]; then
    check_pass "Jetson setup script exists"
    if [ -x "src/autonombot_firmware/scripts/jetson_setup.sh" ]; then
        check_pass "Jetson setup script is executable"
    else
        check_warn "Jetson setup script not executable (run: chmod +x)"
    fi
else
    check_fail "Jetson setup script missing"
fi

if [ -f "src/autonombot_firmware/scripts/validate_hardware.py" ]; then
    check_pass "Hardware validation script exists"
else
    check_fail "Hardware validation script missing"
fi

echo -e "${BLUE}🔍 Checking Dependencies...${NC}"

# Check Python requirements
if [ -f "src/autonombot_vision/requirements.txt" ]; then
    check_pass "Vision requirements.txt exists"
    echo "   Required packages:"
    cat src/autonombot_vision/requirements.txt | grep -v "^#" | grep -v "^$" | sed 's/^/     - /'
else
    check_fail "Vision requirements.txt missing"
fi

echo -e "${BLUE}📝 Checking RViz Configuration...${NC}"

# Check RViz files
if [ -f "src/autonombot_launch/rviz/autonombot_simulation.rviz" ]; then
    check_pass "Custom RViz configuration exists"
else
    check_fail "Custom RViz configuration missing"
fi

echo -e "${BLUE}🏗️ Testing Build System...${NC}"

# Test if workspace can build
echo "Testing workspace build..."
if colcon build --packages-select autonombot_launch autonombot_drivers 2>/dev/null; then
    check_pass "Workspace builds successfully"
else
    check_fail "Workspace build errors detected"
fi

echo -e "${BLUE}🔧 Checking URDF Files...${NC}"

# Check URDF files
urdf_files=(
    "src/autonombot_description/urdf/autonombot.urdf.xacro"
    "src/autonombot_description/urdf/autonombot_gazebo.xacro"
)

for urdf_file in "${urdf_files[@]}"; do
    if [ -f "$urdf_file" ]; then
        check_pass "URDF file $(basename $urdf_file) exists"
    else
        check_fail "URDF file $urdf_file missing"
    fi
done

echo -e "${BLUE}📊 Validation Summary${NC}"
echo "=========================================="
echo -e "Total Checks: $((passed + failed))"
echo -e "${GREEN}Passed: $passed${NC}"
echo -e "${RED}Failed: $failed${NC}"

if [ $failed -eq 0 ]; then
    echo -e "${GREEN}🎉 ALL CHECKS PASSED - READY FOR DEPLOYMENT!${NC}"
    echo ""
    echo "Next steps for lab deployment:"
    echo "1. Copy this workspace to USB drive or push to Git"
    echo "2. Transfer to Jetson Xavier"
    echo "3. Run: src/autonombot_firmware/scripts/jetson_setup.sh"
    echo "4. Build workspace: colcon build"
    echo "5. Validate hardware: ros2 run autonombot_firmware validate_hardware.py"
    echo "6. Launch robot: ros2 launch autonombot_launch real_robot.launch.py"
    exit 0
else
    echo -e "${RED}❌ DEPLOYMENT NOT READY - Fix failed checks first${NC}"
    exit 1
fi
