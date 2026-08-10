# Rotating platform encoder data from Teensy
# Used for synchronizing VLP-16 LiDAR with platform rotation

std_msgs/Header header      # Timestamp from Pi (not Teensy)

uint32 teensy_timestamp_ms  # Teensy millis() value
float64 angle_deg           # Platform angle in degrees (0-360)
float32 velocity_rpm        # Platform rotational velocity in RPM
