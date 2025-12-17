export ROS_DOMAIN_ID=99
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

# Start tmux session
SESSION="ros2_nodes"
tmux new-session -d -s $SESSION

# Define nodes to run
nodes=(
  "user_interface"
  "aruco_pose_node"
  "lidar"
  "move_robot_follow_path"
  "move_robot_outside_box"
  "move_robot_to_aruco"
  "move_robot"
  "process_lidar"
  "rotate_robot_to_aruco"
  "rotate_robot"
  "robot_manager"
  "navigator"
  "localizer"
  "aruco_detector"
)

# Create a window for each node
for i in "${!nodes[@]}"; do
  if [ $i -eq 0 ]; then
    # First node runs in the first tmux window
    tmux rename-window -t $SESSION:0 "${nodes[$i]}"
    tmux send-keys -t $SESSION:0 "ros2 run final_pkg ${nodes[$i]}" C-m
  else
    tmux new-window -t $SESSION -n "${nodes[$i]}"
    tmux send-keys -t $SESSION:$i "ros2 run final_pkg ${nodes[$i]}" C-m
  fi
done

# Attach to the tmux session
tmux attach -t $SESSION
