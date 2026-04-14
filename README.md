# Marble_Balancing_Robotic_Arm


ros2 topic pub --once /target_position geometry_msgs/msg/Pose "{
  position: {x: 0.6, y: 0.433, z: 0.7}, 
  orientation: {x: 0.0, y: 0.1305, z: 0.0, w: 0.9914}
}"




ros2 launch Marble_control marble_view.launch.py
