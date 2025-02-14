#! /bin/bash

if [ -f /proc/xenomai/version ];then
	EXEC=./Camille_rt
else
	EXEC=./Camille_nrt
fi

$EXEC -n Drone_0 -a 127.0.0.1 -p 9000 -l /tmp -x setup_x4.xml -t x4_simu -g car -u 10000
