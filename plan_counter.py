#!/usr/bin/env python
# -*- coding: utf-8 -*-
import filecmp 
import fnmatch
import hashlib
import os
import re
import sys

import collections

def main():
    plan_frequency(sys.argv[1])

def plan_frequency(directory):
    ascending = True
    num_plans = len(fnmatch.filter(os.listdir(directory), 'sas_plan.*'))
    plan_cost = -1
    plan_costs = [-1]*(num_plans)
    for i in range(0, num_plans):
        planfile = directory + '/sas_plan.'+str(i+1)
        plan_cost = search_for_pattern(planfile)
        plan_costs[i] = plan_cost
    counter=collections.Counter(plan_costs)
    print counter


def search_for_pattern(filename):
    pattern = ""
    with open(filename) as f:
        for line in f:
            if "cost" in line:
                pattern = re.findall(r'\d+', str(line))
            if len(pattern) > 0:
                return pattern[0]


if __name__ == "__main__":
    main()
