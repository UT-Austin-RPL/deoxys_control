#!/bin/bash

. $(dirname "$0")/color_variables.sh
. $(dirname "$0")/fix_ld_issue.sh

while true
do
    bin/gripper-interface $@
    sleep 0.1
done
