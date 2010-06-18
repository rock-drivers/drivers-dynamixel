#!/bin/bash

for baud in 9600 19200 38400 57600 115200 230400; do
	echo $baud
	for id in `seq 0 1 253`; do
		#echo $baud $id
		./dynamixel_test -port /dev/ttyUSB0 -id $id -baud $baud -connect &> search.log
		if [ $? == 0 ];then
			echo year! $baud $id
		fi
	done
done
