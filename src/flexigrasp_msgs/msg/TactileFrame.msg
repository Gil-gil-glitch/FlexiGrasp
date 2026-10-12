# One tactile frame from one FlexiTac finger pad (16 x 32 taxels, 8-bit ADC).
#
# header.stamp    host (system clock) time at which the complete frame was received
# header.frame_id sensor frame, e.g. "left_finger_tactile"

std_msgs/Header header

string finger                # "left" or "right" (the name given to the finger in the node config)
uint32 seq                   # frames received from this finger since the node started (rejected frames are not counted)

uint8 rows                   # 16
uint8 cols                   # 32
uint8[] raw                  # rows*cols raw ADC values, row-major (index = row*cols + col)

bool calibrated              # false while the baseline is still being collected (delta/features are zero)
float32[] delta              # raw - baseline, row-major, masked taxels forced to 0

# Features computed from delta (masked taxels excluded)
float32 peak                 # largest delta over all taxels (counts)
float32 total                # sum over taxels of max(delta - contact_threshold, 0)
uint16 contact_taxels        # number of taxels with delta > contact_threshold
bool in_contact              # contact_taxels >= contact_min_taxels
float32 centroid_row         # pressure-weighted centroid in taxel units; NaN if no contact
float32 centroid_col         # NaN if no contact

# Link health (cumulative since node start)
uint32 bad_frames            # frames rejected because a byte was lost or added in transit
uint32 dropped_bytes         # bytes discarded while re-synchronising
float32 rate_hz              # measured frame rate over the last second
