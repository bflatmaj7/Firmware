# Inertial navigation system data (orientation, speed, etc).

float32 ax		# Acceleration in x-axis
float32 ay		# Acceleration in y-axis
float32 az		# Acceleration in z-axis
float32 gx		# Gyro rate in x-axis
float32 gy		# Gyro rate in y-axis
float32 gz		# Gyro rate in z-axis
float32 mx		# Magnetic field in x-axis
float32 my		# Magnetic field in y-axis
float32 mz		# Magnetic field in z-axis

float32 roll		# Roll angle reading
float32 pitch		# Pitch angle reading
float32 yaw		# Yaw angle reading

float32 vns		# North-South velocity
float32 vew		# East-West velocity
float32 vud		# Up-Down velocity

uint32 sol_status	# solution status
uint16 imu_status	# imu status

uint32 id		# unique device ID for the sensor that does not change between power cycles
