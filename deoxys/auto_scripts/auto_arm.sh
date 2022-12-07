#!/bin/bash
. $(dirname "$0")/color_variables.sh
. $(dirname "$0")/fix_ld_issue.sh

printf "${BIRed} Make sure you are in the Performance Mode!!! ${Color_Off} \n"

RTOS_MODE=$(cat /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor)

echo $RTOS_MODE

for mode in $RTOS_MODE
do
    if [ "${mode}" = "powersave" ]; then
	printf "${BIRed} Not in Performance Mode, will cause errors for franka codebase!!! ${Color_Off} \n"
    fi
done

while true
do
    bin/franka-interface $@
    sleep 1
done
