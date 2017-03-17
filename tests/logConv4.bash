#!/bin/bash

echo "Initialize permissions"
source ../catkin_ws/devel/setup.bash

#file="2015-10-28-13-49-04.bag"
#f="${file%.*}"

for file in *.bag
do
	f="${file%.*}"
	echo "processing" $file" file.."
	echo "imu readings"
	imutxt="_IMU.txt"
	rostopic echo -b $file -p /imu_readings > $f$imutxt
	echo "position readings"
	postxt="_POS.txt"
	rostopic echo -b $file -p /pos_readings > $f$postxt
	echo "remote readings"
	remtxt="_REMOTE.txt"
	rostopic echo -b $file -p /remote_readings > $f$remtxt
	echo "gps readings"
        gpstxt="_GPS.txt"
        rostopic echo -b $file -p /gps_readings > $f$gpstxt
	echo "speed readings"
	remtxt="_CONTROL.txt"
	rostopic echo -b $file -p /control_readings > $f$remtxt
	
	echo "creating folder"
	mkdir $f
	ext="_*"
	mv $file $f/
	mv $f$ext $f/
	echo "done"
done

echo "All files converted !"
