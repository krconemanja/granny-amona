# DDBOAT python3 drivers version 2

The drivers are :
* IMU (MAG, ACCEL, GYRO) : imu9_driver_v2.py
* GPS (serial line, GPGLL message) : gps_driver_v2.py
* Arduino motors command (serial line) : arduino_driver_v2.py
* (Used for monitoring only) Simple send/receive protocol with LORA radio : radio_driver_V2.py


## testing the drivers on your ddboat

First, you will have to clone this repo on your laptop in a folder called **drivers** 
```
$ git clone https://gitlab.ensta-bretagne.fr/zerrbe/drivers-ddboat-v2.git drivers
```

This section should be rewritten to adapt with DDBoats V2 (todo)

### simplify access to ddboats

To avoid typing password every time, a ssh keys pair can be created :
```
cd ~/.ssh
ssh-keygen -t rsa
```
Filename can be ddboat_key and no passphrase is provided.
After creation the public key has to be copied on the DDBOAT with XX the boat number on 2 digit (e.g 07 for boat 7)
```
scp ddboat_key.pub ue32@172.20.25.2XX:.ssh
```
add the content of ddboat_key.pub at the end of **authorized_keys** (in ~/.ssh) using vi or nano.

Note : if **authorized_keys** does not exist in .ssh, just create it as an empty file :
```
touch authorized_keys
```


## using the screen command to work outside of the WIFI Network

When using WIFI, the terminal is locked when the WIFI connection is lost.
The ddboat wil stop working properly.
A solution is to use the **screen** command to detach the session from the terminal.
First, we start a screen session called sesddboat (or whatsoever name)

```
$ screen -S sesddboat
```
Note that screen creates a new terminal (and you loose the command history for recent commands)
We can now check that the session is attached
```
$ screen -ls
```
should give :
```
There is a screen on:
        1404.sesddboat     (09/29/21 16:42:04)     (Attached)
1 Socket in /run/screen/S-pi.

```
The screen commands start with Ctrl+A. You can see a list of usefull commands here :

https://linuxize.com/post/how-to-use-linux-screen/

Now, we can detach the screen session from the terminal by typing Ctrl+A d
and we can check that the session is actually detached :
```
$ screen -ls
```
should give :
```
There is a screen on:
	1404.sesddboat	(09/29/21 16:42:04)	(Detached)
1 Socket in /run/screen/S-pi.
```

Now the ddboat can work outside the WIFI coverage. To get access back (resume access) to the session when the ddboat is back in WIFI area, type :
```
screen -r
```
we are back on line with the ddboat !

A last usefull command is to fully stop the screen command, one way to do it is :

```
$ screen -X -S sesddboat quit
```

where sesddboat is the session name the has been used at the start.

Example (log the GPS in a gpx file)
```
$ screen -S sesgps
$ python3 tst_gpx.py (in the drivers-ddboat-v2 folder)
$ CTRL+A d  
now you can go outside the WIFI area and acquire a GPS track
when it's done, recover the session
$ screen -r
$ CTRL+C to stop the acquisition
a file called tst.gpx has been created, rename it to save the data
$ mv tst.gpx mynicetrack.gpx
we can now destroy the session
$ screen -X -S  sesgps quit
```


## working with GPS

The following packets must have been installed :
```
$ sudo apt install python3-gpxpy python3-pyproj
```

The GPS is acquired and the GPGLL message is recorded in GPX format (so you can plot it in GeoPortail for example) in the **tst.gpx** file with the command:
```
$ python3 tst_gpx.py
```

In the NMEA message GPGLL, the latitude and longitude are given in DDmm.mmmm with DD in degrees and mm.mmmm in decimal minutes. To get the value in decimal degrees we do DD + mm.mmmm/60.0

Using a projection from degrees (longitude,latitude in WGS84) to meters (UTM zone 30N) we can compute the distance in meter to a reference point. The following command executes a test:
```
$ python3 tst_proj.py
```

The gpx file can be display on **GeoPortail**. To show it on **GoogleEarth** the simpliest way is to convert it in kml :
```
$ gpsbabel -i gpx -f file.gpx -o kml -F file.kml
```

## remote install drivers for tests (for system user only)

Verify that the arduino code is the right one before updating a DDBOAT.

From a remote computer use rcp and rsh to update, example for DDBOAT 5
```
$ scp install.bash pi@172.20.25.205:/tmp/
$ rsh 172.20.25.205 -l pi /tmp/install.bash
```




