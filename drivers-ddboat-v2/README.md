# DDBoat drivers (v2)

Hardware drivers for the DDBoat, provided by the ENSTA Bretagne teaching staff
for the course. They are not our work; they are included here so the project
runs as is. Original source:
<https://gitlab.ensta-bretagne.fr/zerrbe/drivers-ddboat-v2>

Only the drivers this project uses are kept:

| File | Hardware |
| --- | --- |
| `arduino_driver_v2.py` | Motor commands to the Arduino (serial) |
| `gps_driver_v2.py` | GPS, NMEA GPGLL messages (serial) |
| `imu9_driver_v2.py` | 9-axis IMU: magnetometer, accelerometer, gyroscope (I2C) |
| `i2creal.py` | I2C access, used by the IMU driver |

They need `pyserial` and `smbus`, and only work on the boat.
