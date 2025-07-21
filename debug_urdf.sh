#!/bin/bash

echo "=== URDF Debugging Script ==="

# Generate the robot description manually
echo "1. Generating robot description manually..."
cd ~/airship_simulation_ros_jazzy
source install/setup.bash

xacro src/blimp_description/urdf/blimp_base.xacro \
  enable_meshes:=false \
  enable_wind:=false \
  enable_physics:=false \
  enable_sensors:=false \
  enable_logging:=false \
  enable_ground_truth:=false \
  enable_mavlink_interface:=false \
  enable_custom_plugins:=false \
  log_file:=blimp \
  wait_to_record_bag:=false \
  uav_name:=blimp \
  namespace:=blimp \
  is_input_joystick:=false \
  > /tmp/generated_urdf.xml

echo "2. URDF Analysis:"
echo "   File size: $(wc -c < /tmp/generated_urdf.xml) bytes"
echo "   Links found: $(grep -c '<link name=' /tmp/generated_urdf.xml)"
echo "   Joints found: $(grep -c '<joint name=' /tmp/generated_urdf.xml)"

echo "3. All links:"
grep '<link name=' /tmp/generated_urdf.xml | sed 's/.*name="\([^"]*\)".*/   - \1/'

echo "4. Fixed vs other joints:"
echo "   Fixed joints: $(grep -c 'type="fixed"' /tmp/generated_urdf.xml)"
echo "   Revolute joints: $(grep -c 'type="revolute"' /tmp/generated_urdf.xml)"
echo "   Continuous joints: $(grep -c 'type="continuous"' /tmp/generated_urdf.xml)"

echo "5. Check for problematic joints with zero limits:"
grep -A5 -B5 'lower="0".*upper="0"' /tmp/generated_urdf.xml | grep 'joint name='

echo "6. Generated URDF saved to: /tmp/generated_urdf.xml"
