#!/bin/bash

echo "=== Gazebo Plugin Verification Script ==="

# Function to check if Gazebo is running
check_gazebo() {
    if pgrep -x "gz" > /dev/null; then
        echo "✓ Gazebo is running"
        return 0
    else
        echo "✗ Gazebo is not running"
        return 1
    fi
}

# Function to list Gazebo topics
list_gz_topics() {
    echo -e "\n--- Gazebo Topics ---"
    gz topic -l
}

# Function to check specific plugin topics
check_plugin_topics() {
    echo -e "\n--- Checking Plugin Topics ---"
    
    topics=(
        "/test/wind_force"
        "/test/wind_speed" 
        "/test/dynamic_volume"
        "/test/helium_mass"
    )
    
    gz_topics=$(gz topic -l)
    
    for topic in "${topics[@]}"; do
        if echo "$gz_topics" | grep -q "$topic"; then
            echo "✓ Found: $topic"
            # Echo one message from the topic
            echo "  Sample: $(gz topic -e -t $topic -n 1 2>/dev/null | head -n 5)"
        else
            echo "✗ Missing: $topic"
        fi
    done
}

# Function to check loaded plugins
check_loaded_plugins() {
    echo -e "\n--- Checking Loaded Plugins ---"
    
    # Get model info
    gz_output=$(gz model --list 2>&1)
    echo "Models in simulation: $gz_output"
    
    # Check if our test model exists
    if echo "$gz_output" | grep -q "test_blimp"; then
        echo "✓ Test model 'test_blimp' is loaded"
    else
        echo "✗ Test model 'test_blimp' not found"
    fi
}

# Function to test publishing to plugin
test_plugin_interaction() {
    echo -e "\n--- Testing Plugin Interaction ---"
    echo "Publishing test helium mass value..."
    
    # Publish a test message
    gz topic -t /test/helium_mass -m gz.msgs.Float -p 'data: 1.5' &
    PUB_PID=$!
    sleep 2
    kill $PUB_PID 2>/dev/null
    
    echo "Check if dynamic volume changed in response"
}

# Main verification flow
main() {
    if ! check_gazebo; then
        echo "Please start Gazebo with the test world first:"
        echo "  cd ~/airship_simulation_ros_jazzy && source install/setup.bash"
        echo "  ros2 launch blimp_gazebo_plugin test_plugins_launch.py"
        exit 1
    fi
    
    list_gz_topics
    check_plugin_topics
    check_loaded_plugins
    test_plugin_interaction
    
    echo -e "\n--- ROS2 Bridge Check ---"
    echo "ROS2 Topics (if bridge is running):"
    ros2 topic list | grep test || echo "No ROS2 bridge topics found"
}

# Run main function
main