#! /bin/bash

if [ -f /proc/xenomai/version ];then
	EXEC=./DWA_traj1_ugv_rt
else
	EXEC=./DWA_traj1_ugv_nrt
fi

$EXEC -n ugv_0 -t sumo -a 127.0.0.1 -p 9000 -l /tmp -x setup_sumo.xml
