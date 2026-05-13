std_msgs/Header header

string target_id
int8 mode
uint8 track_state

bool evaluated
bool valid
bool fire_advice

int32 best_candidate_index
float64 yaw_error
float64 pitch_error
bool best_candidate_facing_ok

int32 candidate_count_total
int32 candidate_count_facing_eligible
int32 candidate_count_facing_rejected

bool probability_enabled
float64 p_hit_window
float64 fire_score
float64 best_tau_ms
float64 e_u
float64 e_v
float64 sigma_u
float64 sigma_v
float64 armor_width_m
float64 armor_height_m
float64 burst_probability
float64 log_evidence
float64 evidence_sum
float64 evidence_strength
int32 gate_strategy
int32 gate_state
