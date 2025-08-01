float64 timestamp

bool environment_outputted
bool attitude_outputted
bool mag_cal_outputted
bool acc_cal_outputted
bool ahrs_raw_data_outputted
bool ahrs_comp_data_outputted

float32 env_supply
float32 env_temperature
float32 env_pressure
float32 env_depth
float32 env_vos

float32 attitude_yaw
float32 attitude_pitch
float32 attitude_roll

uint8 mag_cal_buf
bool mag_cal_valid
uint32 mag_cal_age
uint8 mag_cal_fit

int16[3] acceleration_limit_min
int16[3] acceleration_limit_max

int16[3] ahrs_raw_acc
int16[3] ahrs_raw_mag
int16[3] ahrs_raw_gyro

geometry_msgs/Vector3 ahrs_comp_acc
geometry_msgs/Vector3 ahrs_comp_mag
geometry_msgs/Vector3 ahrs_comp_gyro
