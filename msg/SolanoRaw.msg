# Uninterpreted bytes returned by a 12-byte Solano I2C read.
# No command/register write precedes the read. No scaling or decoding is applied.

uint64 timestamp                 # [us] Publication time since system start
uint64 timestamp_sample          # [us] Time before starting the I2C read
uint32 device_id                 # PX4 bus/address/device identifier
uint32 error_count               # Cumulative failed reads
uint8[12] data                  # Bytes in received order
