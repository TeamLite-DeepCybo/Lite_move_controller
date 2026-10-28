# Full canonical batch, SI units (rad/m). Opt-in bounded stream ingress only.
std_msgs/Header header
string stream_id
string command_id
uint64 sequence
# Reset interpolation to this measured hold target on transition into hold.
bool hold
string[] joint_names
float64[] positions
