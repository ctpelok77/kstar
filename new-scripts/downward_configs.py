"""
Example configurations taken from
http://alfons.informatik.uni-freiburg.de/downward/PlannerUsage
"""
import sys
import logging
from collections import defaultdict

import experiments
import tools
import downward_suites


HELP = """\
Comma separated list of configurations. They can be specified in the following \
ways: ou, astar_searches, myconfigfile:yY, myconfigfile:lama_configs.
The python modules have to live in the scripts dir.
"""

# Eager A* search with landmark-cut heuristic (previously configuration ou)
ou = '--search "astar(lmcut())"'

fF = """\
--heuristic "hff=ff()" \
--search "lazy_greedy(hff, preferred=(hff))"\
"""

yY = """\
--heuristic "hcea=cea()" \
--search "lazy_greedy(hcea, preferred=(hcea))"\
"""

yY_eager = """\
--heuristic "hcea=cea()" \
--search "eager_greedy(hcea, preferred=(hcea))"\
"""

fFyY = """\
--heuristic "hff=ff()" --heuristic "hcea=cea()" \
--search "lazy_greedy(hff, hcea, preferred=(hff, hcea))"\
"""

lama = """\
--heuristic "hlm,hff=lm_ff_syn(lm_rhw(reasonable_orders=true,lm_cost_type=2,cost_type=2))" \
--search "iterated(lazy_greedy(hff,hlm,preferred=(hff,hlm)),\
lazy_wastar(hff,hlm,preferred=(hff,hlm),w=5),\
lazy_wastar(hff,hlm,preferred=(hff,hlm),w=3),\
lazy_wastar(hff,hlm,preferred=(hff,hlm),w=2),\
lazy_wastar(hff,hlm,preferred=(hff,hlm),w=1),\
repeat_last=true)"\
"""

lama_noreas = """\
--heuristic "hlm,hff=lm_ff_syn(lm_rhw(reasonable_orders=false,lm_cost_type=2,cost_type=2))" \
--search "iterated(lazy_greedy(hff,hlm,preferred=(hff,hlm)),\
lazy_wastar(hff,hlm,preferred=(hff,hlm),w=5),\
lazy_wastar(hff,hlm,preferred=(hff,hlm),w=3),\
lazy_wastar(hff,hlm,preferred=(hff,hlm),w=2),\
lazy_wastar(hff,hlm,preferred=(hff,hlm),w=1),\
repeat_last=true)"\
"""

lama_unit = """\
--heuristic "hlm,hff=lm_ff_syn(lm_rhw(reasonable_orders=true,lm_cost_type=1,cost_type=1))" \
--search "iterated(lazy_greedy(hff,hlm,preferred=(hff,hlm),cost_type=1),\
lazy_wastar(hff,hlm,preferred=(hff,hlm),w=5,cost_type=1),\
lazy_wastar(hff,hlm,preferred=(hff,hlm),w=3,cost_type=1),\
lazy_wastar(hff,hlm,preferred=(hff,hlm),w=2,cost_type=1),\
lazy_wastar(hff,hlm,preferred=(hff,hlm),w=1,cost_type=1),\
repeat_last=true)"\
"""

lama_noreas_unit = """\
--heuristic "hlm,hff=lm_ff_syn(lm_rhw(reasonable_orders=false,lm_cost_type=1,cost_type=1))" \
--search "iterated(lazy_greedy(hff,hlm,preferred=(hff,hlm),cost_type=1),\
lazy_wastar(hff,hlm,preferred=(hff,hlm),w=5,cost_type=1),\
lazy_wastar(hff,hlm,preferred=(hff,hlm),w=3,cost_type=1),\
lazy_wastar(hff,hlm,preferred=(hff,hlm),w=2,cost_type=1),\
lazy_wastar(hff,hlm,preferred=(hff,hlm),w=1,cost_type=1),\
repeat_last=true)"\
"""

lama_newhybrid = """\
--heuristic "hlm1,hff1=lm_ff_syn(lm_rhw(reasonable_orders=false,lm_cost_type=1,cost_type=1))" \
--heuristic "hlm2,hff2=lm_ff_syn(lm_rhw(reasonable_orders=false,lm_cost_type=2,cost_type=2))" \
--search "iterated(lazy_greedy(hff1,hlm1,preferred=(hff1,hlm1),cost_type=1),\
lazy_greedy(hff2,hlm2,preferred=(hff2,hlm2)),\
lazy_wastar(hff2,hlm2,preferred=(hff2,hlm2),w=5),\
lazy_wastar(hff2,hlm2,preferred=(hff2,hlm2),w=3),\
lazy_wastar(hff2,hlm2,preferred=(hff2,hlm2),w=2),\
lazy_wastar(hff2,hlm2,preferred=(hff2,hlm2),w=1),\
repeat_last=true)"\
"""

lama_noreas_hybrid = """\
--heuristic "hlm1,hff1=lm_ff_syn(lm_rhw(reasonable_orders=false,lm_cost_type=1,cost_type=1))" \
--heuristic "hlm2,hff2=lm_ff_syn(lm_rhw(reasonable_orders=false,lm_cost_type=2,cost_type=2))" \
--search "iterated(lazy_greedy(hff1,hlm1,preferred=(hff1,hlm1),cost_type=1),\
lazy_wastar(hff1,hlm1,preferred=(hff1,hlm1),w=5,cost_type=1),\
lazy_wastar(hff1,hlm1,preferred=(hff1,hlm1),w=3,cost_type=1),\
lazy_wastar(hff1,hlm1,preferred=(hff1,hlm1),w=2,cost_type=1),\
lazy_wastar(hff1,hlm1,preferred=(hff1,hlm1),w=1,cost_type=1),\
lazy_wastar(hff2,hlm2,preferred=(hff2,hlm2),w=1,cost_type=0),\
repeat_last=true)"\
"""

blind = """\
--search "astar(blind())"\
"""

oa50000 = """\
--search "astar(mas())"\
"""

oa200000 = """\
--search "astar(mas(max_states=200000))"\
"""

pdb1000 = """\
--search "astar(pdb(max_states=1000))"\
"""

pdb2500 = """\
--search "astar(pdb(max_states=2500))"\
"""

pdb5000 = """\
--search "astar(pdb(max_states=5000))"\
"""

pdb10000 = """\
--search "astar(pdb(max_states=10000))"\
"""

pdb25000 = """\
--search "astar(pdb(max_states=25000))"\
"""

pdb50000 = """\
--search "astar(pdb(max_states=50000))"\
"""

pdb100000 = """\
--search "astar(pdb(max_states=100000))"\
"""

pdb250000 = """\
--search "astar(pdb(max_states=250000))"\
"""

pdb500000 = """\
--search "astar(pdb(max_states=500000))"\
"""

pdb1000000 = """\
--search "astar(pdb(max_states=1000000))"\
"""

pdb2500000 = """\
--search "astar(pdb(max_states=2500000))"\
"""

pdb5000000 = """\
--search "astar(pdb(max_states=5000000))"\
"""

pdb10000000 = """\
--search "astar(pdb(max_states=10000000))"\
"""

pdb25000000 = """\
--search "astar(pdb(max_states=25000000))"\
"""

pdb50000000 = """\
--search "astar(pdb(max_states=50000000))"\
"""

pdb100000000 = """\
--search "astar(pdb(max_states=100000000))"\
"""

lmopt_rhw = """\
--search "astar(lmcount(lm_rhw(),admissible=true),mpd=true)"\
"""

lmopt_hm1 = """\
--search "astar(lmcount(lm_hm(m=1),admissible=true),mpd=true)"\
"""

lmopt_zg = """\
--search "astar(lmcount(lm_zg(),admissible=true),mpd=true)"\
"""

lmopt_rhw_hm1 = """\
--search "astar(lmcount(lm_merged(lm_rhw(),lm_hm(m=1)),admissible=true),mpd=true)"\
"""

lmopt_rhw_zg = """\
--search "astar(lmcount(lm_merged(lm_rhw(),lm_zg()),admissible=true),mpd=true)"\
"""

lmopt_hm1_zg = """\
--search "astar(lmcount(lm_merged(lm_zg(),lm_hm(m=1)),admissible=true),mpd=true)"\
"""

lmopt_rhw_hm1_zg = """\
--search "astar(lmcount(lm_merged(lm_rhw(),lm_zg(),lm_hm(m=1)),admissible=true),mpd=true)"\
"""


iter_ff = """\
--heuristic "h=ff(cost_type=1)" \
--search "iterated(lazy_greedy(h, preferred=(h)), repeat_last=true)"\
"""


def astar_searches():
    return [('blind', blind), ('oa50000', oa50000)]

def arch_comp_configs():
    return [('blind', blind), ('oa200000', oa200000), ('yY', yY),
            ('yY_eager', yY_eager)]


def get_old_and_new_greedy(pairs):
    return pairs + [(nick.replace('lg', 'og'),
        config.replace('lazy_greedy', 'old_greedy')) for nick, config in pairs]


def issue154a():
    return get_old_and_new_greedy([
        ('lg_blind', '--search "lazy_greedy(blind())"'),
        ('lg_ff', '--search "lazy_greedy(ff())"'),
        ('lg_cea', '--search "lazy_greedy(cea())"'),
        ('lg_ff_cea', '--search "lazy_greedy(ff(), cea())"'),])

def issue154b():
    return get_old_and_new_greedy([
        ('lg_hff', '--heuristic "hff=ff()" '
            '--search "lazy_greedy(hff, preferred=(hff))"'),
        ('lg_hcea', '--heuristic "hcea=cea()" '
            '--search "lazy_greedy(hcea, preferred=(hcea))"'),
        ('lg_hff_hcea', '--heuristic "hff=ff()" --heuristic "hcea=cea()" '
            '--search "lazy_greedy(hff, hcea, preferred=(hff, hcea))"'),
        ('lg_hlm_hff', '--heuristic "hlm,hff=lm_ff_syn(lm_rhw())" '
            '--search "lazy_greedy(hlm, hff, preferred=(hlm, hff))"'
),])



def get_configs(configs_strings):
    """
    Parses configs_strings and returns a list of tuples of the form
    (configuration_name, configuration_string)

    config_strings can contain strings of the form
    "configs.py:cfg13" or "configs.py"
    """
    all_configs = []

    files_to_configs = defaultdict(list)
    for config_string in configs_strings:
        if ':' in config_string:
            config_file, config_name = config_string.split(':')
        else:
            # Check if this module has the config
            config_file, config_name = __file__, config_string

        files_to_configs[config_file].append(config_name)

    for file, config_names in files_to_configs.iteritems():
        module = tools.import_python_file(file)
        module_dict = module.__dict__
        for config_name in config_names:
            config_or_func = module_dict.get(config_name, None)
            if config_or_func is None:
                msg = 'Config "%s" could not be found in "%s"' % (config_name, file)
                logging.error(msg)
                sys.exit()
            try:
                config_list = config_or_func()
            except TypeError:
                config_list = [(config_name, config_or_func)]

            all_configs.extend(config_list)

    logging.info('Found configs: %s' % all_configs)
    return all_configs


if __name__ == '__main__':
    get_configs(['blind', 'downward_configs:astar_searches'])

