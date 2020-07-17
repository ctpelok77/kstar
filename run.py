#! /usr/bin/env python

import json, sys
import string
import subprocess

def os_mv(from_file, to_file):
    subprocess.call(["mv", from_file, to_file], shell=False)


def main(numPlans, domain, problem):
    planner_args = ["--search", "astar(blind(),dump_forbid_plan_reformulation=true)"]
    first_run_args = ["./fast-downward.py", domain, problem]
    subprocess.call(first_run_args + planner_args, shell=False)

    os_mv("reformulated_output.sas", "reformulated_output.sas.1")
    os_mv("output.sas", "reformulated_output.sas.0")
    os_mv("sas_plan", "sas_plan_1")

    for i in range(1,numPlans):
        next = str(i + 1)
        next_run_args = ["./fast-downward.py", 'reformulated_output.sas.%s' % str(i)]
        subprocess.call(next_run_args + planner_args, shell=False)
        os_mv("sas_plan", "sas_plan_%s" % next)
        os_mv("reformulated_output.sas", "reformulated_output.sas.%s" % next)


if __name__ == "__main__":
    main(int(sys.argv[1]), sys.argv[2], sys.argv[3])
