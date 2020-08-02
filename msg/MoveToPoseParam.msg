# Selection matrix is determined based on fd
PoseMsg task_frame
PoseMsg target_pose
float64[6] fd              # desired force
float64 speed_factor
float64 timeout
franka_controllers/Gain controller_gain
