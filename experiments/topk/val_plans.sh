#!/bin/bash

# run multiplan validator externally; this step is now embedded inside parser
script=/data/downward-projects/validate-multiplan-solutions/validate_multiplan_solutions/validate_run.py

for folder in data/kstar-first-v1_09-04-2022/runs-*/*
do 
  echo validate plans in $folder
  python $script --planner topk --number-of-plans 100 --run-folder $folder --plans-folder found_plans
done

