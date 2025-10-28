# Message that contains intermediate results 
# and diagnostics of the (predictive) planner.

std_msgs/Header header

# The planned trajectory (or if multiple plans exist, all of them)
cohan_msgs/Trajectory[] trajectories

# Index of the trajectory in 'trajectories' that is selected currently
uint16 selected_trajectory_idx

# List of active obstacles
costmap_converter_msgs/ObstacleArrayMsg obstacles_msg


