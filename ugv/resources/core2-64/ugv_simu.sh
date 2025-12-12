#! /bin/bash

if [ -f /proc/xenomai/version ];then
	EXEC=./DWA_traj_ugv_rt
else
	EXEC=./DWA_traj_ugv_nrt
fi

$EXEC -n ugv_0 -t ugv_simu -a 127.0.0.1 -p 9000 -l /tmp -x setup_simu.xml
