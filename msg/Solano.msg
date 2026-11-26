# Decoded measurements from the Chronics Solano airflow probe.

uint64 timestamp                 # [us] Publication time since system start
uint64 timestamp_sample          # [us] Time before starting the I2C read
uint32 device_id                 # PX4 bus/address/device identifier
float32 windspeed                # [m/s] Axial airflow speed
float32 windspeed_f              # [m/s] Filtered axial airflow speed
float32 angle_ns                 # [deg] NS airflow angle in the probe frame
float32 angle_ns_f               # [deg] Filtered NS airflow angle
float32 angle_ew                 # [deg] EW airflow angle in the probe frame
float32 angle_ew_f               # [deg] Filtered EW airflow angle
