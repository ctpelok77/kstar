#!/bin/bash

# $1 domain
# $2 problem
# $3 quality bound (q)
# $4 upper bound on the number of plans (k)
# $5 json file name

if [ "$#" -ne 5 ]; then
    echo "Illegal number of parameters"
    exit 1
fi

RUNOPT="kstar(blind(),q=$3,k=$4,json_file_to_dump=$5)"

#LOG_FILE=run.log

SOURCE="$( dirname "${BASH_SOURCE[0]}" )"
#echo $SOURCE
$SOURCE/fast-downward.py $1 $2 --search $RUNOPT 
#$SOURCE/fast-downward.py $1 $2 --search $RUNOPT > $LOG_FILE

