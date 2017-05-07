#! /usr/bin/env python

import json,sys; 
import string
import subprocess


def main(numPlans, domain, problem):
    subprocess.call(["./fast-downward.py", domain, problem, "--search", "forbid_astar(blind())"], shell=False)

    subprocess.call(["mv", "reformulated_output.sas", "reformulated_output.sas.1"], shell=False)
    subprocess.call(["mv", "output.sas", "reformulated_output.sas.0"], shell=False)
    subprocess.call(["mv", "sas_plan", "sas_plan_1"], shell=False)

    for i in range(1,numPlans):
	next = str(i + 1)
	subprocess.call(["./fast-downward.py", 'reformulated_output.sas.%s' % str(i), "--search", "forbid_astar(blind())"], shell=False)
    	subprocess.call(["mv", "sas_plan", "sas_plan_%s" % next], shell=False)
    	subprocess.call(["mv", "reformulated_output.sas", "reformulated_output.sas.%s" % next], shell=False)


if __name__ == "__main__":
   main(int(sys.argv[1]), sys.argv[2], sys.argv[3])
