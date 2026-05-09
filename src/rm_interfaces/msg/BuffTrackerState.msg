std_msgs/Header header

# Robot identity: "big_buff" or "small_buff"
string robot_id

# Track state enum
uint8 LOST=0
uint8 TEMP_LOST=1
uint8 TRACKING=2
uint8 track_state

# Center position in world frame (odom)
geometry_msgs/Point center_position

# Center roll angle (radians). 0 = blade pointing up, positive = CCW
float64 center_roll

# Center roll angular velocity (rad/s)
float64 center_vroll

# Blade inactivation flags — true means blade already hit (thus ENGAGEABLE)
bool[5] blade_inactivated

# Number of blades
int32 num_blades

# Engageable mask (LSB=blade 0), bit=1 means engageable
uint32 engageable_mask

# Estimate confidence [0, 1]
float64 confidence

# Big buff curve-fit parameters (NaN when small buff or not fitted)
float64 big_a
float64 big_omega
float64 big_c
float64 big_d
float64 big_dt_from_start

# Monotonic sequence number for diagnostics
uint32 state_seq
