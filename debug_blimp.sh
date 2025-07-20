#!/bin/bash
# debug_blimp.sh - Complete debugging script for blimp migration issues

echo "========================================"
echo "    BLIMP ROS2 MIGRATION DEBUGGER"
echo "========================================"

# Set environment
source install/setup.bash

echo ""
echo "=== 1. CHECKING PLUGIN FILES ==="
echo "Looking for built plugin files..."

plugin_files=$(find install/ -name "*.so" | grep blimp)
if [ -z "$plugin_files" ]; then
    echo "❌ NO PLUGIN FILES FOUND!"
    echo "   Need to build blimp_gazebo_plugin package:"
    echo "   colcon build --packages-select blimp_gazebo_plugin"
    plugins_exist=false
else
    echo "✅ Plugin files found:"
    echo "$plugin_files"
    plugins_exist=true
fi

echo ""
echo "=== 2. CHECKING ENVIRONMENT VARIABLES ==="

echo "GAZEBO_PLUGIN_PATH:"
if echo $GAZEBO_PLUGIN_PATH | grep -q blimp; then
    echo "✅ Contains blimp paths"
    echo "$GAZEBO_PLUGIN_PATH" | tr ':' '\n' | grep blimp
else
    echo "❌ No blimp paths found"
fi

echo ""
echo "GZ_SIM_SYSTEM_PLUGIN_PATH:"
if echo $GZ_SIM_SYSTEM_PLUGIN_PATH | grep -q blimp; then
    echo "✅ Contains blimp paths"
    echo "$GZ_SIM_SYSTEM_PLUGIN_PATH" | tr ':' '\n' | grep blimp
else
    echo "❌ No blimp paths found"
fi

echo ""
echo "LD_LIBRARY_PATH (blimp entries):"
if echo $LD_LIBRARY_PATH | grep -q blimp; then
    echo "✅ Contains blimp paths"
    echo "$LD_LIBRARY_PATH" | tr ':' '\n' | grep blimp
else
    echo "❌ No blimp paths found"
fi

echo ""
echo "=== 3. TESTING URDF GENERATION ==="

# Test URDF generation with minimal plugins
echo "Testing URDF generation (minimal plugins)..."
xacro_output=$(xacro src/blimp_description/urdf/blimp_base.xacro \
  enable_meshes:=false \
  enable_wind:=false \
  enable_physics:=false \
  enable_sensors:=false \
  enable_logging:=false \
  enable_ground_truth:=false \
  enable_mavlink_interface:=false \
  log_file:=test \
  wait_to_record_bag:=false \
  uav_name:=blimp \
  namespace:=blimp \
  is_input_joystick:=false 2>&1)

if [ $? -eq 0 ]; then
    echo "✅ URDF generation successful (minimal)"
    echo "$xacro_output" > /tmp/blimp_minimal.urdf
else
    echo "❌ URDF generation failed (minimal):"
    echo "$xacro_output"
    exit 1
fi

# Test URDF generation with plugins enabled (if plugins exist)
if [ "$plugins_exist" = true ]; then
    echo ""
    echo "Testing URDF generation (with physics plugins)..."
    xacro_output_full=$(xacro src/blimp_description/urdf/blimp_base.xacro \
      enable_meshes:=false \
      enable_wind:=false \
      enable_physics:=true \
      enable_sensors:=false \
      enable_logging:=false \
      enable_ground_truth:=false \
      enable_mavlink_interface:=false \
      log_file:=test \
      wait_to_record_bag:=false \
      uav_name:=blimp \
      namespace:=blimp \
      is_input_joystick:=false 2>&1)
    
    if [ $? -eq 0 ]; then
        echo "✅ URDF generation successful (with physics)"
        echo "$xacro_output_full" > /tmp/blimp_physics.urdf
    else
        echo "❌ URDF generation failed (with physics):"
        echo "$xacro_output_full"
    fi
fi

echo ""
echo "=== 4. CHECKING PLUGIN USAGE IN URDF ==="

if [ -f "/tmp/blimp_minimal.urdf" ]; then
    echo "Checking minimal URDF..."
    
    # Check for ROS2 control
    if grep -q "ros2_control" /tmp/blimp_minimal.urdf; then
        echo "✅ ros2_control block found"
    else
        echo "❌ ros2_control block missing"
    fi
    
    if grep -q "gz_ros2_control" /tmp/blimp_minimal.urdf; then
        echo "✅ gz_ros2_control plugin found"
    else
        echo "❌ gz_ros2_control plugin missing"
    fi
    
    # Check motor model syntax
    if grep -q "actuator_number" /tmp/blimp_minimal.urdf; then
        echo "✅ Modern motor syntax (actuator_number) found"
    else
        echo "⚠️  actuator_number not found"
    fi
    
    if grep -q "motorNumber" /tmp/blimp_minimal.urdf; then
        echo "❌ Deprecated motor syntax (motorNumber) found - needs fixing"
    else
        echo "✅ No deprecated motor syntax found"
    fi
fi

if [ -f "/tmp/blimp_physics.urdf" ] && [ "$plugins_exist" = true ]; then
    echo ""
    echo "Checking physics URDF for custom plugins..."
    
    custom_plugins=("libnormwind_plugin" "libfinliftdrag_plugin" "libdynamicvolume_plugin")
    for plugin in "${custom_plugins[@]}"; do
        if grep -q "$plugin" /tmp/blimp_physics.urdf; then
            echo "✅ $plugin found in URDF"
        else
            echo "❌ $plugin missing from URDF"
        fi
    done
fi

echo ""
echo "=== 5. LAUNCH FILE DIAGNOSTICS ==="

launch_files=("spawn_uav.launch.py" "blimp_only.launch.py")
for launch_file in "${launch_files[@]}"; do
    launch_path="src/blimp_description/launch/$launch_file"
    if [ -f "$launch_path" ]; then
        echo "Checking $launch_file..."
        
        # Check for problematic patterns
        if grep -q "on_exit" "$launch_path"; then
            echo "❌ Contains problematic 'on_exit' parameter"
        else
            echo "✅ No problematic 'on_exit' parameter"
        fi
        
        if grep -q "respawn.*True" "$launch_path"; then
            echo "⚠️  Contains respawn=True (might cause multiple spawning)"
        else
            echo "✅ No problematic respawn found"
        fi
        
        if grep -q "robot_description.*topic" "$launch_path"; then
            echo "✅ robot_description topic handling found"
        else
            echo "⚠️  robot_description topic handling not clear"
        fi
    else
        echo "❌ $launch_file not found"
    fi
done

echo ""
echo "=== 6. RECOMMENDATIONS ==="

if [ "$plugins_exist" = false ]; then
    echo "🔧 IMMEDIATE ACTION REQUIRED:"
    echo "   1. Build the plugin package:"
    echo "      colcon build --packages-select blimp_gazebo_plugin"
    echo "   2. Source the environment:"
    echo "      source install/setup.bash"
    echo "   3. Re-run this debug script"
fi

if ! echo $GZ_SIM_SYSTEM_PLUGIN_PATH | grep -q blimp; then
    echo "🔧 ENVIRONMENT ISSUE:"
    echo "   Plugin paths not set correctly."
    echo "   Check blimp_gazebo_plugin.dsv.in file and rebuild package."
fi

echo ""
echo "=== 7. TEST COMMANDS ==="
echo "To test minimal configuration (no custom plugins):"
echo "   ros2 launch blimp_description blimp_only.launch.py enable_physics:=false"
echo ""
echo "To test with physics (if plugins work):"
echo "   ros2 launch blimp_description blimp_only.launch.py enable_physics:=true"
echo ""
echo "Generated URDF files for inspection:"
echo "   /tmp/blimp_minimal.urdf"
if [ -f "/tmp/blimp_physics.urdf" ]; then
    echo "   /tmp/blimp_physics.urdf"
fi

echo ""
echo "========================================"
echo "Debug complete. Check recommendations above."
echo "========================================"