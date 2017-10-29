#!/usr/bin/env python
# -*- coding: utf-8 -*-
import json
import pprint
import os
import re
import filecmp 
import hashlib
import fnmatch
from util import *
from subprocess import Popen, PIPE 

def ascending_order(directory, dict):
	not_in_ascending_order = []
	error = False 
	for key in dict:
		# weird hack
		if "iterative" in directory:
			plan_dir = directory+"/"+str(dict[key][1])+"/found_plans/done"
		if  "kstar" in directory: 		
			plan_dir = directory+"/"+str(dict[key][1])+"/found_plans"

		if (os.path.exists(plan_dir)):
			ascending_order = in_ascending_order(plan_dir)
			if not ascending_order:
				not_in_ascending_order.append(key)
				error = True
	if error:
		print "[ERR] Not in ascending_order!"	
		print "Directory " + str(directory)
		print not_in_ascending_order

def validate_plan_frequencies(dict_first_approach, dict_second_approach):
	for key in dict_first_approach:
		plan_freq1 = dict_first_approach[key][0] 	
		plan_freq2 = dict_second_approach[key][0] 
		for i in range(0, min(len(plan_freq1), len(plan_freq2))):
			if (i >= len(plan_freq1) - 1 and i >= len(plan_freq2) - 1): 
				continue
			if (plan_freq1[i] == -1 or plan_freq2[i]  == -1):
				continue
			if (int(plan_freq1[i]) < int(plan_freq2[i])): 	
				print "[ERR] K* Problem?"
				print "iterative_approach"+"/"+dict_first_approach[key][1]+"/found_plans/done/sas_plan."+str(i) 
				print "kstar/" + dict_second_approach[key][1] +"/found_plans/sas_plan."+ str(i)  
				print "sas_plan." + str(i+1) + " Iterative Approach plan costs:"+str(plan_freq1[i])+ " K* plan costs: " + str(plan_freq2[i])
				print	key 
				print  ""
				break	

			if (int(plan_freq1[i]) > int(plan_freq2[i])): 	
				print "[ERR] Iterative Approach Problem?"
				print "iterative_approach"+"/"+dict_first_approach[key][1]+"/found_plans/done/sas_plan."+str(i) 
				print "kstar/" + dict_second_approach[key][1] +"/found_plans/sas_plan."+ str(i)
				print "sas_plan." + str(i) + " Iterative Approach plan costs:"+str(plan_freq1[i])+ " K* plan costs: " + str(plan_freq2[i])
				print  key
				print ""
				break	

runs_iterative_approach = enumerate_runs(1700)
print "Create dictionary iterative_approach..."
iterative_approach =  create_dictionary("iterative_approach",runs_iterative_approach) 
runs_kstar = enumerate_runs(2600)
print "Create dictionary K*..."
kstar =  create_dictionary("kstar",runs_kstar)

#print "Checking ascending plan order iterative_approach..."
#ascending_order("iterative_approach", iterative_approach)
#print "Checking ascending plan order K*..."
#ascending_order("kstar", kstar)

print "Checking plan frequencies..."
validate_plan_frequencies(iterative_approach, kstar)

# this is intractable 
#print "Checking duplicates iterative_approach..."
#check_duplicates("iterative_approach", iterative_approach)
#print "Checking duplicates K*..."
#check_duplicates("kstar", kstar)

