#! /bin/bash

if [ -f /proc/xenomai/version ];then
	EXEC=./Camille_rt
else
	EXEC=./Camille_nrt
fi

$EXEC -n Drone_0 -a autodetect -p 9000 -l /tmp -x setup_x4_intelaero.xml -t aero -g car -u 10000
