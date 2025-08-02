# Steering angle message for Ranger robot
# Contains steering angle in both radians and degrees

std_msgs/Header header

# Steering angle values
float64 angle_rad    # Steering angle in radians
float64 angle_deg    # Steering angle in degrees

# Optional: individual wheel angles for 4-wheel steering
float64 front_left_rad    # Front left wheel angle in radians
float64 front_right_rad   # Front right wheel angle in radians
float64 rear_left_rad     # Rear left wheel angle in radians  
float64 rear_right_rad    # Rear right wheel angle in radians

float64 front_left_deg    # Front left wheel angle in degrees
float64 front_right_deg   # Front right wheel angle in degrees
float64 rear_left_deg     # Rear left wheel angle in degrees
float64 rear_right_deg    # Rear right wheel angle in degrees
