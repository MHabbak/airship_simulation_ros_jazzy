#!/bin/bash
# message_compatibility_check.sh - Check for message compatibility issues

echo "============================================"
echo "    MESSAGE COMPATIBILITY CHECKER"
echo "============================================"

# Check for C++ includes that need updating
echo "=== 1. CHECKING C++ INCLUDES ==="
echo "Looking for mav_msgs includes that need updating to msgs..."

if find src/ -name "*.cpp" -o -name "*.hpp" -o -name "*.h" | xargs grep -l "mav_msgs" 2>/dev/null; then
    echo "⚠️  Found C++ files with mav_msgs includes:"
    find src/ -name "*.cpp" -o -name "*.hpp" -o -name "*.h" | xargs grep -l "mav_msgs" 2>/dev/null
    echo ""
    echo "These files need to be updated:"
    echo "  #include <mav_msgs/...>  →  #include <msgs/...>"
    needs_cpp_update=true
else
    echo "✅ No C++ files with mav_msgs includes found"
    needs_cpp_update=false
fi

echo ""
echo "=== 2. CHECKING TOPIC REFERENCES ==="
echo "Looking for topic names that reference mav_msgs..."

if find src/ -name "*.cpp" -o -name "*.hpp" -o -name "*.py" -o -name "*.launch.py" -o -name "*.xacro" | xargs grep -l "mav_msgs/" 2>/dev/null; then
    echo "⚠️  Found files with mav_msgs/ topic references:"
    find src/ -name "*.cpp" -o -name "*.hpp" -o -name "*.py" -o -name "*.launch.py" -o -name "*.xacro" | xargs grep -l "mav_msgs/" 2>/dev/null
    echo ""
    echo "These might need updating if they reference topic names"
    needs_topic_update=true
else
    echo "✅ No topic references found"
    needs_topic_update=false
fi

echo ""
echo "=== 3. CHECKING MESSAGE COMPATIBILITY ==="
echo "Comparing your migrated messages with expected usage..."

# Check AttitudeThrust.msg difference
echo "AttitudeThrust.msg:"
if grep -q "geometry_msgs/Quaternion attitude" src/msgs/msg/AttitudeThrust.msg 2>/dev/null; then
    echo "✅ Uses Quaternion for attitude (good - more precise than Vector3)"
else
    echo "⚠️  Check AttitudeThrust.msg format"
fi

# Check for any missing message files
expected_msgs=("Actuators.msg" "AttitudeThrust.msg" "RateThrust.msg" "RollPitchYawrateThrust.msg" "TorqueThrust.msg" "Status.msg" "FilteredSensorData.msg" "GpsWaypoint.msg")

echo ""
echo "Required message files:"
for msg in "${expected_msgs[@]}"; do
    if [ -f "src/msgs/msg/$msg" ]; then
        echo "✅ $msg found"
    else
        echo "❌ $msg missing"
    fi
done

echo ""
echo "=== 4. DEPENDENCY CHECK ==="
echo "Checking if blimp_gazebo_plugin depends on msgs..."

if grep -q "<depend>msgs</depend>" src/blimp_gazebo_plugin/package.xml 2>/dev/null; then
    echo "✅ blimp_gazebo_plugin depends on msgs"
else
    echo "❌ blimp_gazebo_plugin missing msgs dependency"
    echo "   Add <depend>msgs</depend> to blimp_gazebo_plugin/package.xml"
    needs_dependency_update=true
fi

if grep -q "<depend>msgs</depend>" src/blimp_description/package.xml 2>/dev/null; then
    echo "✅ blimp_description depends on msgs"
else
    echo "⚠️  Consider adding msgs dependency to blimp_description/package.xml"
fi

echo ""
echo "=== 5. MESSAGE GENERATION CHECK ==="
if [ -f "src/msgs/CMakeLists.txt" ]; then
    if grep -q "rosidl_generate_interfaces" src/msgs/CMakeLists.txt; then
        echo "✅ msgs package configured for message generation"
    else
        echo "❌ msgs package missing rosidl_generate_interfaces"
    fi
else
    echo "❌ msgs/CMakeLists.txt not found"
fi

echo ""
echo "============================================"
echo "    SUMMARY & RECOMMENDATIONS"
echo "============================================"

if [ "$needs_cpp_update" = true ]; then
    echo "🔧 ACTION REQUIRED: Update C++ includes"
    echo "   Find and replace: #include <mav_msgs/ → #include <msgs/"
fi

if [ "$needs_topic_update" = true ]; then
    echo "🔧 ACTION REQUIRED: Check topic references"
    echo "   Update any topic names from mav_msgs/ to msgs/"
fi

if [ "$needs_dependency_update" = true ]; then
    echo "🔧 ACTION REQUIRED: Add msgs dependency"
    echo "   Add <depend>msgs</depend> to blimp_gazebo_plugin/package.xml"
fi

echo ""
echo "🎯 BUILD SEQUENCE:"
echo "1. Fix any issues above"
echo "2. colcon build --packages-select msgs"
echo "3. source install/setup.bash"
echo "4. colcon build --packages-select blimp_gazebo_plugin"
echo "5. colcon build --packages-select blimp_description"