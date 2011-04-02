#include "globals.h"

#include <cstdlib>
#include <iostream>
#include <limits>
#include <string>
#include <vector>
#include <unistd.h>
using namespace std;

#include "axioms.h"
#include "cache.h"
#include "causal_graph.h"
#include "domain_transition_graph.h"
#include "operator.h"
#include "state.h"
#include "successor_generator.h"
#include "timer.h"

bool test_goal(const State &state) {
    for(int i = 0; i < g_goal.size(); i++) {
        if(state[g_goal[i].first] != g_goal[i].second) {
            return false;
        }
    }
    return true;
}

bool peek_magic(istream &in, string magic) {
  string word;
  in >> word;
  bool result = (word == magic);
  in.putback('\n');
  for(int i = word.size() - 1; i >= 0; i--)
    in.putback(word[i]);
  return result;
}

void check_magic(istream &in, string magic) {
  string word;
  in >> word;
  if(word != magic) {
    cout << "Failed to match magic word '" << magic << "'." << endl;
    cout << "Got '" << word << "'." << endl;
    exit(1);
  }
}

void read_metric(istream &in) {
  check_magic(in, "begin_metric");
  in >> g_use_metric;
  check_magic(in, "end_metric");
}

void read_variables(istream &in) {
  check_magic(in, "begin_variables");
  int count;
  in >> count;
  for(int i = 0; i < count; i++) {
    string name;
    in >> name;
    g_variable_name.push_back(name);
    int range;
    in >> range;
    g_variable_domain.push_back(range);
    if(range > numeric_limits<state_var_t>::max()) {
      cout << "You bet!" << endl;
      exit(1);
    }
    int layer;
    in >> layer;
    g_axiom_layers.push_back(layer);
  }
  check_magic(in, "end_variables");
}

void read_goal(istream &in) {
  check_magic(in, "begin_goal");
  int count;
  in >> count;
  for(int i = 0; i < count; i++) {
    int var, val;
    in >> var >> val;
    g_goal.push_back(make_pair(var, val));
  }
  check_magic(in, "end_goal");
}

void dump_goal() {
  cout << "Goal Conditions:" << endl;
  for(int i = 0; i < g_goal.size(); i++)
    cout << "  " << g_variable_name[g_goal[i].first] << ": "
	 << g_goal[i].second << endl;
}

void read_operators(istream &in) {
  int count;
  in >> count;
  for(int i = 0; i < count; i++){
    g_operators.push_back(Operator(in, false));
    g_operators[i].set_index(i);
  }
}

void read_axioms(istream &in) {
  int count;
  in >> count;
  for(int i = 0; i < count; i++){
    g_axioms.push_back(Operator(in, true));
    g_axioms[i].set_index(g_operators.size()+i);
  }
  g_axiom_evaluator = new AxiomEvaluator;
  g_axiom_evaluator->evaluate(*g_initial_state);
}

void read_everything(istream &in) {
//  if(peek_magic(in, "begin_metric")) {
    read_metric(in);
    g_legacy_file_format = false;
//  } else {
//    g_use_metric = false;
//    g_legacy_file_format = true;
//  }
  read_variables(in);
  g_initial_state = new State(in);
  read_goal(in);
  read_operators(in);
  read_axioms(in);
  check_magic(in, "begin_SG");
  g_successor_generator = read_successor_generator(in);
  check_magic(in, "end_SG");
  DomainTransitionGraph::read_all(in);
  g_causal_graph = new CausalGraph(in);
  g_cache = new Cache;
}

void dump_everything() {
  cout << "Use metric? " << g_use_metric << endl;
  cout << "Min Action Cost: " << g_min_action_cost << endl;
  cout << "Variables (" << g_variable_name.size() << "):" << endl;
  for(int i = 0; i < g_variable_name.size(); i++)
    cout << "  " << g_variable_name[i]
	 << " (range " << g_variable_domain[i] << ")" << endl;
  cout << "Initial State:" << endl;
  g_initial_state->dump();
  dump_goal();
  /*
  cout << "Successor Generator:" << endl;
  g_successor_generator->dump();
  for(int i = 0; i < g_variable_domain.size(); i++)
    g_transition_graphs[i]->dump();
  */
}

/*
 * getexename - Get the filename of the currently running executable
 *
 * The getexename() function copies an absolute filename of the currently
 * running executable to the array pointed to by buf, which is of length size.
 *
 * If the filename would require a buffer longer than size elements, NULL is
 * returned, and errno is set to ERANGE; an application should check for this
 * error, and allocate a larger buffer if necessary.
 *
 * Return value:
 * NULL on failure, with errno set accordingly, and buf on success. The
 * contents of the array pointed to by buf is undefined on error.
 *
 * Notes:
 * This function is tested on Linux only. It relies on information supplied by
 * the /proc file system.
 * The returned filename points to the final executable loaded by the execve()
 * system call. In the case of scripts, the filename points to the script
 * handler, not to the script.
 * The filename returned points to the actual exectuable and not a symlink.
 *
 */
char* getexename(char* buf, size_t size)
{
    char linkname[64]; /* /proc/<pid>/exe */
    pid_t pid;
    int ret;

    /* Get our PID and build the name of the link in /proc */
    pid = getpid();

    if (snprintf(linkname, sizeof(linkname), "/proc/%i/exe", pid) < 0)
    {
        /* This should only happen on large word systems. I'm not sure
           what the proper response is here.
           Since it really is an assert-like condition, aborting the
           program seems to be in order. */
        abort();
    }


    /* Now read the symbolic link */
    ret = readlink(linkname, buf, size);

    /* In case of an error, leave the handling up to the caller */
    if (ret == -1)
        return NULL;

    /* Report insufficient buffer size */
    if (ret >= size)
    {
        return NULL;
    }

    /* Ensure proper NUL termination */
    buf[ret] = 0;

    return buf;
}



bool g_legacy_file_format = false; // TODO: Can rip this out after migration.
bool g_use_metric;
int g_min_action_cost = numeric_limits<int>::max();
vector<string> g_variable_name;
vector<int> g_variable_domain;
vector<int> g_axiom_layers;
vector<int> g_default_axiom_values;
State *g_initial_state;
vector<pair<int, int> > g_goal;
vector<Operator> g_operators;
vector<Operator> g_axioms;
AxiomEvaluator *g_axiom_evaluator;
SuccessorGenerator *g_successor_generator;
vector<DomainTransitionGraph *> g_transition_graphs;
CausalGraph *g_causal_graph;
Cache *g_cache;
int g_cache_hits = 0, g_cache_misses = 0;
vector<Abstraction *> g_abstractions;
int g_abstraction_max_size = 1000;
int g_abstraction_peak_memory = 0;
int g_abstraction_nr = 1;
int g_compose_strategy = COMPOSE_LINEAR_CG_GOAL_LEVEL;
int g_collapse_strategy = COLLAPSE_HIGH_F_LOW_H;
bool g_merge_and_shrink_bound_is_for_product = true;

// TODO: The following three should be command-line options.
bool g_merge_and_shrink_simplify_labels = true;
bool g_merge_and_shrink_extra_statistics = false;
bool g_merge_and_shrink_forbid_merge_across_buckets = false;

/* WARNING: When last tested (around r3011), enabling the extra statistics
   for merge-and-shrink increased heuristic generation time by 76%.
   So don't activate it without good reason!

   TODO: Once there is a command-line option for this, move this warning
   to the place where it is set, or even turn it into a message printed to
   cout.
*/

Timer g_timer;

FFHeuristic *g_ff_heur = 0;
SearchSpace* g_learning_search_space = 0;
bool g_do_path_dependent_search = false;
