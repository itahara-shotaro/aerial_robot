# Thrust and gimbal targets the flight controller is applying (spinal -> PC), sent at 200 Hz while control is active
time stamp
float32[] thrust        # target thrust magnitude of each rotor [N], before pwm conversion and duty clamping
float32[] gimbal_angle  # target angle of each gimbal joint [rad], gimbal_dof entries per rotor, ordered by rotor
