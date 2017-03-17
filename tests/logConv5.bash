#!/bin/bash

echo "Initialize permissions"
source ../catkin_ws/devel/setup.bash


for file in *.bag
do
f="${file%.*}"
echo "processing" $file" file.."
echo "imu readings"
imutxt="_IMU.txt"
rostopic echo -b $file -p /imu_readings > $f$imutxt

echo "mag_readings"
magtxt="_MAG.txt"
rostopic echo -b $file -p /mag_readings > $f$magtxt



echo "creating folder"
mkdir $f
ext="_*"
mv $file $f/
mv $f$ext $f/
echo "done"
done

echo "All files converted !"
