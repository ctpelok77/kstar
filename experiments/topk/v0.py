#! /usr/bin/env python
# -*- coding: utf-8 -*-
"""
Modified parser to execute validate-multiplan-solution
it will record the validation results, PASSED (int 1) or FAILED (int 0)
Do a minitest for validation.
"""

import itertools
import os

from lab.environments import IBMLSFEnvironment

# from lab.reports import Attribute, geometric_mean
# from downward.reports.compare import ComparativeReport

import common_setup
from common_setup import IssueConfig, IssueExperiment

DIR = os.path.dirname(os.path.abspath(__file__))
SCRIPT_NAME = os.path.splitext(os.path.basename(__file__))[0]
BENCHMARKS_DIR = os.environ["DOWNWARD_BENCHMARKS"]
REVISIONS = ["master"]
BUILDS = ["release"]
CONFIG_NICKS = [
    # blind
    ('k-blind', ['--search', 'kstar(blind(),k=1000)']),
    ('k-ipdb', ['--search', 'kstar(cpdbs(patterns=hillclimbing(max_time=60)),k=1000)']),
]
CONFIGS = [
    IssueConfig(
        config_nick,
        config,
        build_options=[build],
        driver_options=["--build", build])
    for build in BUILDS
    for config_nick, config in CONFIG_NICKS
]

SUITE = common_setup.DEFAULT_OPTIMAL_SUITE


#SUITE = ["grid", "blocks", "gripper"]

ENVIRONMENT = IBMLSFEnvironment(
            queue="x86_1h",
            export=["PATH"]
)

exp = IssueExperiment(
    revisions=REVISIONS,
    configs=CONFIGS,
    environment=ENVIRONMENT,
    time_limit="30m",
    memory_limit="8g",
)
exp.set_property("k", 1000)
exp.set_property("planner_time_limit", 1800)
exp.set_property("planner_memory_limit", 8000000)

exp.add_suite(BENCHMARKS_DIR, SUITE)

exp.add_parser(exp.EXITCODE_PARSER)
exp.add_parser(exp.TRANSLATOR_PARSER)
exp.add_parser(exp.SINGLE_SEARCH_PARSER)
exp.add_parser(exp.PLANNER_PARSER)
exp.add_parser("parser.py")

exp.add_step('build', exp.build)
exp.add_step('start', exp.start_runs)
exp.add_fetcher(name='fetch')
# exp.add_parse_again_step()

KSTAR_ATTRIBUTES = [
    # plans
    "num_plans",
    "kstar_coverage",
    "kstar_coverage_ratio",
    # time in sec
    "total_kstar_time",
    "total_astar_time",
    "total_eppsetein_time",
    "first_astar_time",
    "after_goal_astar_time",
    # steps or iterations
    "total_steps",
    "total_astar_steps",
    "total_eppsetein_steps",
    #
    "first_astar_dead_ends",
    "first_astar_evaluated",
    "first_astar_expansions",
    "first_astar_generated",
    "first_astar_reopened",
    "first_astar_evaluations_until_last_jump",
    "first_astar_expansions_until_last_jump",
    "first_astar_generated_until_last_jump",
    "first_astar_reopened_until_last_jump",
    #
    "validation_result"
]

IssueExperiment.DEFAULT_TABLE_ATTRIBUTES += KSTAR_ATTRIBUTES
attributes = IssueExperiment.DEFAULT_TABLE_ATTRIBUTES
# exp.add_comparison_table_step(attributes=attributes)
exp.add_absolute_report_step(attributes=attributes)

exp.run_steps()
