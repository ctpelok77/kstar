#!/bin/bash

# $1 domain
# $2 problem
# $3 number of plans (k)
# $4 json file name



RUNOPT="kstar(blind(),$3,json_file_to_dump=$4)"

LOG_FILE=run.log

SOURCE="$( dirname "${BASH_SOURCE[0]}" )"
echo $SOURCE
$SOURCE/fast-downward.py --build release64 $1 $2 --search $RUNOPT > $LOG_FILE

