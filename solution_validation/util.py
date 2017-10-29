#!/usr/bin/env python
# -*- coding: utf-8 -*-

import json
import pprint
import os
import re
import filecmp 
import hashlib
import fnmatch


def search_for_pattern(filename):
    pattern = ""
    with open(filename) as f:
        for line in f:
            if "cost" in line:
                pattern = re.findall(r'\d+', str(line))
            if len(pattern) > 0:
                return int(pattern[0])

# Enumerate all directories of the kind runs-00001-01000/00001...
def enumerate_runs(num):
	runs = [] 
	from_index = 1 
	to_index = 100
	for i in range(1, (num/100)):
		parent_dir = "runs-" + str(from_index).zfill(5) + "-" + str(to_index).zfill(5)		
		for j in range(from_index, to_index + 1):
			runs.append(parent_dir + "/" + str(j).zfill(5))
		from_index += 100
		to_index += 100
	return runs 

def create_dictionary(directory, runs):
	dictionary = {}
	current_dir = os.path.dirname(__file__)
	for run in runs:
		with open(directory+"/"+run+"/properties") as properties_file:    
			data = json.load(properties_file)	
			key = str(data['domain']+"/"+data['problem'])
			if "iterative" in directory:
				if not os.path.exists(directory+"/"+run+"/found_plans/done"):
					continue
				plans, num_plans = get_plan_costs(directory+"/"+run+"/found_plans/done")
			else:	
				if not os.path.exists(directory+"/"+run+"/found_plans"):
					continue
				plans, num_plans = get_plan_costs(directory+"/"+run+"/found_plans")
			dictionary[key] = (plans, run,data['domain'],data['problem'])
	return dictionary

def get_plan_costs(directory):
	num_plans = len(fnmatch.filter(os.listdir(directory), 'sas_plan.*'))
	plan_cost = -1
	plan_costs = [-1]*(num_plans + 1)
	for i in range(1, num_plans):
		planfile = directory + '/sas_plan.'+str(i)
		plan_cost = search_for_pattern(planfile)
		plan_costs[i] = int(plan_cost)
	return plan_costs, num_plans

def in_ascending_order(directory):
	ascending = True
	plan_costs, num_plans = get_plan_costs(directory) 
	old_val = plan_costs[0]
	filename = "sas_plan."
	for i in range(1, num_plans):
		current_val = plan_costs[i]
		if current_val < old_val:
			print "[ERR] " + filename+str(i) +" has less cost than "+ filename+str(i-1)   
			ascending = False
	return ascending

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

def md5(fname):
    hash_md5 = hashlib.md5()
    with open(fname, "rb") as f:
        for chunk in iter(lambda: f.read(4096), b""):
            hash_md5.update(chunk)
    return hash_md5.hexdigest()
