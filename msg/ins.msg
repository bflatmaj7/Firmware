# Inertial navigation system data (orientation, speed, etc).

uint32 clk_time # Ellipse clock timestamp
uint16 year		# Year
uint8 month		# Month
uint8 day		# Day
uint32 tow		# Time of week
uint32 imu_time # IMU message timestamp
float32 ax		# Acceleration in x-axis
float32 ay		# Acceleration in y-axis
float32 az		# Acceleration in z-axis
float32 gx		# Gyro rate in x-axis
float32 gy		# Gyro rate in y-axis
float32 gz		# Gyro rate in z-axis
float32 dvelx	# Sculling output in x-axis
float32 dvely	# Sculling output in y-axis
float32 dvelz	# Sculling output in z-axis
float32 dangx	# Coning output in x-axis
float32 dangy	# Coning output in y-axis
float32 dangz	# Coning output in z-axis
uint32 mag_time	# Mag message time stamp
float32 mx		# Magnetic field in x-axis
float32 my		# Magnetic field in y-axis
float32 mz		# Magnetic field in z-axis

uint32 eul_time	# Euler message time stamp
float32 roll	# Roll angle reading
float32 pitch	# Pitch angle reading
float32 yaw		# Yaw angle reading
float32 roll_acc# Roll angle accuracy
float32 pitch_acc	# Pitch angle accuracy
float32 yaw_acc	# Yaw angle accuracy

uint32 nav_time	# Nav message time stamp
float32 vns		# North-South velocity
float32 vew		# East-West velocity
float32 vud		# Up-Down velocity
float32 vns_acc	# North-South velocity accuracy
float32 vew_acc	# East-West velocity accuracy
float32 vud_acc	# Up-Down velocity accuracy

uint32 gps_hdt_time	# GPS HDT message time stamp
uint32 gps_vel_time	# GPS VEL message time stamp
float32 gps_heading	# GPS Heading
float32 gps_pitch	# GPS Pitch
float32 gps_course	# GPS Course
float32 gps_heading_acc	# GPS Heading, accuracy
float32 gps_pitch_acc	# GPS Pitch, accuracy
float32 gps_course_acc	# GPS Course, accuracy
float32 gps_vns		# GPS North-South velocity
float32 gps_vew		# GPS East-West velocity
float32 gps_vud		# GPS Up-Down velocity
float32 gps_vns_acc	# GPS North-South velocity, accuracy
float32 gps_vew_acc	# GPS East-West velocity, accuracy
float32 gps_vud_acc	# GPS Up-Down velocity, accuracy

uint32 sol_status	# solution status
uint16 imu_status	# imu status
uint16 gps_hdt_status	# gps hdt status
uint32 gps_vel_status	# gps vel status

uint32 id		# unique device ID for the sensor that does not change between power cycles
