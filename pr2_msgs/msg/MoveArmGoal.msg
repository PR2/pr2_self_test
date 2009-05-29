Header header

# Is the goal implicit, defined by constraints?  If so, then the goal is
# constructed from goal_constraints.  Otherwise, it's taken directly from
# configuration
byte implicit_goal

# The desired joint configuration
robot_msgs/JointState[] goal_configuration

# The goal state for the model to plan for. The state is not explicit.
# The goal is considered achieved if all the constraints are satisfied.
motion_planning_msgs/PoseConstraint[] goal_constraints

# Indicate if the goal is being enabled or disabled
byte enable

# Total timelimit for the controller, if a plan is not
# found by this time, abort. Set it to zero for unlimited.
float64 timeout
