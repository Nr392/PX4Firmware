uint64 timestamp	# time since system start (microseconds)
uint32 device_id	# unique device ID for the sensor that does not change between power cycles

uint64 error_count

uint8 key		# WINGLET key

float32 w		# WINGLET W axis
float32 x		# WINGLET X axis
float32 y		# WINGLET Y axis
float32 z		# WINGLET Z axis

bool  is_external	# if true the mag is external (i.e. not built into the board)


