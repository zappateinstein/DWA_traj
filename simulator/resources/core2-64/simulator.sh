#! /bin/bash
if [ -f /proc/xenomai/version ];then
	EXEC=./DWA_traj_simulator_rt
else
	EXEC=./DWA_traj_simulator_nrt
fi

$EXEC -n ugv_0 -p 9000 -a 127.0.0.1 -x simulator.xml -o 10 -m $FLAIR_ROOT/flair-src/models -s $FLAIR_ROOT/flair-src/models/indoor_flight_arena.xml
