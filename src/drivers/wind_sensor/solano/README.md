# Solano

On boards with Solano automatic startup configured, set `SENS_EN_SOLANO` to
`1` in QGroundControl and reboot. The default is `0` (disabled). The board's
startup script selects the I²C bus and address, and retries once after a failed
start. Startup errors are reported in the console; use `solano status` to check
whether the driver is running. Replace `BUS` and `ADDRESS` in the commands below
with the I²C bus and address configured for your board.

```bash
# Check whether the driver started
solano status -X -b BUS -a ADDRESS

# Start manually
solano start -X -b BUS -a ADDRESS

# Dump raw bytes in hex
solano dump -X -b BUS -a ADDRESS

# Show the latest raw and decoded samples
listener solano_raw
listener solano
listener sensor_airflow

# Stop
solano stop -X -b BUS -a ADDRESS
```

`dump` makes a fresh I²C read while the driver is running. The `listener`
commands show samples from the driver's polling loop.

## I²C payload

The driver reads 12 bytes per sample with no preceding register or command
write. It decodes six signed big-endian 16-bit values and multiplies each by
`0.01`. The fields are:

| Byte offset | Field |
| --- | --- |
| 0–1 | `windspeed` |
| 2–3 | `windspeed_f` |
| 4–5 | `angle_ns` |
| 6–7 | `angle_ns_f` |
| 8–9 | `angle_ew` |
| 10–11 | `angle_ew_f` |

For each successful poll, the driver publishes all 12 bytes in received order
to `solano` and `solano_raw`, and publishes filtered speed and angles on
`sensor_airflow`. All topics carry the same timestamp and device ID. The I²C
transfer reports success or failure. It provides no measurement of the probe’s
internal buffer occupancy.

The 12-byte payload contains no sensor status field. The driver sets
`sensor_airflow.status` to `0`; this is not a probe-reported health value. Failed
I²C reads increment the driver's communication error counter and do not publish
a new sample.

## Speed and angle conventions

The probe's vector calculation converts the NS and EW angles from degrees to
radians. Before applying the attitude and mounting-angle transformation, it
uses these probe-frame components.

| Component | Value |
| --- | --- |
| Forward | `windspeed` |
| Lateral | `windspeed × tan(angle_ew)` |
| Vertical | `windspeed × tan(angle_ns)` |

The calculation repeats using `windspeed_f`, `angle_ew_f`, and `angle_ns_f` for
the filtered stream. It treats the speed as the forward-axis component.
Telemetry forwards the transformed components without further speed scaling,
indicating that the decoded speed is in m/s.
[MAVLink WIND_COV field definitions](https://mavlink.io/en/messages/common.html#WIND_COV)

The probe's vector calculation assigns `angle_ew` to the lateral component and
`angle_ns` to the vertical component. The current `sensor_airflow` mapping below
follows the separately provided convention, where NS is the horizontal
`direction` and EW is the vertical `elevation`. Verify this axis mapping and
sign convention on the physical probe before estimator use.

`sensor_airflow.speed` is the filtered `windspeed_f` value.
`sensor_airflow.direction` carries the filtered NS angle and the added
`sensor_airflow.elevation` field carries the filtered EW angle. Both angles are
converted from degrees to radians. The speed is passed through unchanged. The
topic carries speed and angles, not a Cartesian velocity vector, and EKF2 does
not currently subscribe to it.

The vector conversion provided for Solano is:

```text
velocity_x = speed × cos(direction) × cos(elevation)
velocity_y = speed × sin(direction)
velocity_z = speed × sin(elevation)
```

This conversion uses the total amplitude as `speed`. The probe's vector
calculation instead treats `windspeed_f` as the forward component and multiplies
it by the tangents of the angles. These are different speed conventions. The
topic currently publishes `windspeed_f` unchanged, so its interpretation as
total amplitude or forward component must be settled before converting it to a
Cartesian vector or using it as true airspeed.

## Logging

`sensor_airflow` is included in the default logger topic set at 10 Hz. The
`SDLOG_SOLANO` parameter controls the additional Solano topics; it takes effect
after restarting the logger (the parameter is marked reboot-required).

| `SDLOG_SOLANO` | Solano data logged |
| --- | --- |
| 0 | `sensor_airflow` only (default) |
| 1 | `sensor_airflow` and decoded `solano` at 10 Hz |
| 2 | `sensor_airflow`, decoded `solano` at 10 Hz, and full-rate `solano_raw` |

EKF2 currently does not consume `sensor_airflow`. The airspeed-to-wind
estimation path consumes `airspeed` or `airspeed_validated` instead.
