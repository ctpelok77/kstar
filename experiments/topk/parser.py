#! /usr/bin/env python
# -*- coding: utf-8 -*-


import re
import glob, os, sys
import json
import itertools

from lab.parser import Parser

print("Parsing with custom parser")

_PLAN_INFO_REGEX = re.compile(r"; cost = (\d+) \((unit cost|general cost)\)\n")

def get_plan_cost(path):
    try:
        with open(path) as input_file:
            line = None
            for line in input_file:
                if line.strip().startswith(";"):
                    continue
            # line is now the last line
            match = _PLAN_INFO_REGEX.match(line)
            if match:
                return int(match.group(1))
            return None
    except:
        return None


def get_plan_costs(plans_folder):
    ret = []
    for counter in itertools.count(1):
        name = "sas_plan.%d" % counter
        plan_filename = os.path.join(plans_folder, name)
        if not os.path.exists(plan_filename):
            break
        cost = get_plan_cost(plan_filename)
        if cost is not None:
            ret.append(cost)
    return ret

def plans(content, props):
    costs = get_plan_costs('found_plans')
    props["plan_costs"] = costs
    props["num_plans"] = len(costs)



def unsolvable(content):
    p = re.findall(r'Completely explored state space -- no solution', content, re.M)
    return len(p) > 0 or len(q) > 0 


def get_data_from_static():
    with open("static-properties", 'r') as sp:
        return json.load(sp)

def get_k(props):
    return 1000


def coverage(content, props):
    props["coverage"] = int('total_time' in props)


parser = Parser()
parser.add_function(plans)
parser.add_pattern('search_time', r'Search time: (.+)s', required=False, type=float)
parser.add_pattern('total_time', r'Total time: (.+)s', required=False, type=float)
parser.add_pattern('raw_memory', r'Peak memory: (\d+) KB', required=False, type=int)
parser.add_function(coverage)

parser.parse()
