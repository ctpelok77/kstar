#!/usr/bin/env python
# -*- coding: utf-8 -*-

import filecmp 
import fnmatch
import hashlib
import os
import re
import sys
import shutil
import collections

def remove_plan_files():
	pass
	#shutil.rmtree("found_plans",ignore_errors=False, onerror=None) 

def solution_sanity_check():
    directory = "found_plans"
    print "Starting Top-k plan sanity checks..."
    duplicates = exist_duplicates(directory)
    success = True
    if not duplicates:
		print "No duplicate plans."
    else: 
		print "Duplicate plans found."
		success = False 

    ascending = in_ascending_order(directory)
    if ascending:
        print "Plans are in ascending order."
    else:
		print "Plans are not in ascending order."
		success = True

    plan_frequencies("found_plans") 
    return success  

def exist_duplicates(directory):
    print "Checking for duplicate plans..."
    duplicates = False
    for filename1 in os.listdir(directory):
        for filename2 in os.listdir(directory):
            if filename1 == filename2 or filename1 > filename2:
                continue
            f1 = directory+"/"+filename1
            f2 = directory+"/"+filename2
            if md5(f1) == md5(f2):
                duplicates =  True
                print "[ERR] "+ filename1 + " and " + filename2 +" are identical!"  
    return duplicates

def in_ascending_order(directory):
    print "Checking ascending order of the plans..."
    ascending = True
    num_plans = len(fnmatch.filter(os.listdir(directory), 'sas_plan.*'))
    plan_cost = -1
    plan_costs = [-1]*(num_plans + 1)
    for i in range(1, num_plans):
        planfile = directory + '/sas_plan.'+str(i)
        plan_cost = search_for_pattern(planfile)
        plan_costs[i] = plan_cost

    old_val = plan_costs[0]
    filename = "sas_plan."
    for i in range(1, num_plans):
        current_val = plan_costs[i]
        if current_val < old_val:
            print "[ERR] " + filename+str(i) +" has less cost than "+ filename+str(i-1)   
            ascending = False
    return ascending 

def plan_frequencies(directory):
    ascending = True
    num_plans = len(fnmatch.filter(os.listdir(directory), 'sas_plan.*'))
    plan_cost = -1
    plan_costs = [-1]*(num_plans)
    for i in range(0, num_plans):
        planfile = directory + '/sas_plan.'+str(i+1)
        plan_cost = search_for_pattern(planfile)
        plan_costs[i] = plan_cost
    counter=collections.Counter(plan_costs)
    
    f = open('plan_frequencies','w')
    print "(plan-cost, frequency)"
    for key, value in counter.items(): 
		print (key,value)
		line = str(key) + " "+ str(value) + "\n"
		f.write(line)


def search_for_pattern(filename):
    pattern = ""
    with open(filename) as f:
        for line in f:
            if "cost" in line:
                pattern = re.findall(r'\d+', str(line))
            if len(pattern) > 0:
                return pattern[0] 

def md5(fname):
    hash_md5 = hashlib.md5()
    with open(fname, "rb") as f:
        for chunk in iter(lambda: f.read(4096), b""):
            hash_md5.update(chunk)
    return hash_md5.hexdigest()

