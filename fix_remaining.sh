#!/bin/bash
# quick_fix_all_issues.sh - Address the 3 remaining issues

echo "============================================"
echo "    QUICK FIX FOR 3 REMAINING ISSUES"
echo "============================================"

cd ~/airship_simulation_ros_jazzy

echo "=== ISSUE 1: FINDING DEPRECATED MOTOR SYNTAX ==="
echo "Looking for motorNumber that needs to be changed to actuator_number..."

motor_files_to_check=(
    "src/blimp_description/urdf/description_plugin.xacro"
    "src/blimp_description/urdf/plugin_control.xacro" 
    "src/blimp_description/urdf/description_controller.xacro"
)

found_motor_issues=false
for file in "${motor_files_to_check[@]}"; do
    if [ -f "$file" ]; then
        if grep -q "motorNumber" "$file"; then
            echo "❌ Found motorNumber in: $file"
            echo "   Lines containing motorNumber:"
            grep -n "motorNumber" "$file"
            found_motor_issues=true
        fi
    fi
done

if [ "$found_motor_issues" = false ]; then
    echo "✅ No motorNumber syntax found"
else
    echo ""
    echo "🔧 MANUAL FIX REQUIRED:"
    echo "   Replace: <motorNumber>X</motorNumber>"
    echo "   With:    <actuator_number>X</actuator_number>"
fi

echo ""
echo "=== ISSUE 2: CHECKING WIND PLUGIN ==="

# Test wind plugin inclusion
xacro src/blimp_description/urdf/blimp_base.xacro \
  enable_wind:=true enable_physics:=true \
  enable_meshes:=false enable_sensors:=false \
  enable_logging:=false enable_ground_truth:=false \
  enable_mavlink_interface:=false \
  uav_name:=blimp namespace:=blimp \
  is_input_joystick:=false > /tmp/test_wind.urdf 2>/dev/null

if grep -q "libnormwind_plugin" /tmp/test_wind.urdf; then
    echo "✅ Wind plugin found when enable_wind:=true"
    echo "   Issue: You were testing with enable_wind:=false"
else
    echo "❌ Wind plugin missing even with enable_wind:=true"
    echo "   Check: src/blimp_description/urdf/description_plugin.xacro"
fi

echo ""
echo "=== ISSUE 3: LAUNCH FILE PROBLEMS ==="

if grep -q "on_exit" src/blimp_description/launch/spawn_uav.launch.py; then
    echo "❌ spawn_uav.launch.py contains problematic 'on_exit' parameter"
    echo "   Fix: Replace spawn_uav.launch.py with corrected version"
    launch_needs_fix=true
else
    echo "✅ spawn_uav.launch.py looks clean"
    launch_needs_fix=false
fi

echo ""
echo "============================================"
echo "    QUICK FIXES"
echo "============================================"

if [ "$found_motor_issues" = true ]; then
    echo "🔧 FIX 1: Motor Syntax"
    echo "   Edit the files listed above and replace motorNumber with actuator_number"
    echo ""
fi

echo "🔧 FIX 2: Wind Plugin" 
echo "   Use: enable_wind:=true to include wind plugin"
echo "   Test: ros2 launch blimp_description blimp_only.launch.py enable_wind:=true"
echo ""

if [ "$launch_needs_fix" = true ]; then
    echo "🔧 FIX 3: Launch File"
    echo "   Replace spawn_uav.launch.py with the corrected version provided"
    echo ""
fi

echo "============================================"
echo "    TEST COMMANDS (After Fixes)"
echo "============================================"
echo ""
echo "1. Test minimal (should work):"
echo "   ros2 launch blimp_description blimp_only.launch.py enable_physics:=false"
echo ""
echo "2. Test with physics (should work):"
echo "   ros2 launch blimp_description blimp_only.launch.py enable_physics:=true"
echo ""
echo "3. Test with wind (should work after fixes):"
echo "   ros2 launch blimp_description blimp_only.launch.py enable_wind:=true enable_physics:=true"
echo ""
echo "4. Test full configuration:"
echo "   ros2 launch blimp_description blimp_only.launch.py"

echo ""
echo "============================================"
echo "You're 90% there! These are minor fixes."
echo "============================================"