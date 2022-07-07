bool stop               # Whether to forcefully stop
int8 filter_position    # One of the 2 filter positions: 0 or 1
int8 coordinate_mode    # Absolute (0) or relative (1) mode
int8 motor_move_mode    # Normal (0) or forced (1) mode
uint64 wait             # Miliseconds to wait
int32[] steps           # 16 bit steps in both sides
int32[] speed           # Could be int16 but need negative when motor is ignored
int32[] count           # Could be int16 but need negative when motor is ignored
string command          # Custom G-code command