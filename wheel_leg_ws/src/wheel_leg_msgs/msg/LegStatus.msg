# Robot leg status telemetry
float32 left_leg_length    # Left leg current length (meters)
float32 right_leg_length   # Right leg current length (meters)
float32 pitch              # Pitch angle (radians)
float32 roll               # Roll angle (radians)
float32 vx                 # Current forward velocity (m/s)
float32 omega              # Current angular velocity (rad/s)
float32 battery_voltage    # Battery voltage (V), -1 if unavailable
uint8   motor_state        # 0=stopped, 1=running, 2=error
