#!/usr/bin/env python
# -*- coding: utf-8 -*-

import filecmp 
import fnmatch
import hashlib
import os
import re

def main():
    directory = "../found_plans"
    if exist_duplicates(directory):
        pass
    else:
        print "No duplicates."

    if ascending_order(directory):
        pass
    else:
        pass


def exist_duplicates(directory):
    for filename1 in os.listdir(directory):
        for filename2 in os.listdir(directory):
            if filename1 == filename2:
                continue
            f1 = directory+"/"+filename1
            f2 = directory+"/"+filename2
            if md5(f1) == md5(f2):
                print "[ERR] "+ filename1 + " and " + filename2 +" are identical!"  

def ascending_order(directory):
    num_plans = len(fnmatch.filter(os.listdir(directory), 'sas_plan.*'))
    plan_cost = -1
    for i in range(1, num_plans):
        planfile = directory + '/sas_plan.'+str(i)
        plan_cost = search_for_pattern(planfile)
        print plan_cost
         
def search_for_pattern(filename):
    pattern = ""
    with open(filename) as f:
        for line in f:
            pattern = re.compile(r'\d+', line)[0]
    return 0 

def md5(fname):
    hash_md5 = hashlib.md5()
    with open(fname, "rb") as f:
        for chunk in iter(lambda: f.read(4096), b""):
            hash_md5.update(chunk)
    return hash_md5.hexdigest()


if __name__ == "__main__":
    main()
