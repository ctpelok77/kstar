#include "general_eager_best_first_search.h"

#include "globals.h"
#include "heuristic.h"
#include "successor_generator.h"
#include "search_node_info.h"

#include <cassert>
#include <cstdlib>
#include <set>
#include <math.h>
using namespace std;

GeneralEagerBestFirstSearch::GeneralEagerBestFirstSearch(bool reopen_closed):
    reopen_closed_nodes(reopen_closed),
    open_list(0), f_evaluator(0) {


}

GeneralEagerBestFirstSearch::~GeneralEagerBestFirstSearch() {
}

// TODO: changes this to add_open_list,
// including type of open list, use of preferred operators, which heuristic to use, etc.
void GeneralEagerBestFirstSearch::add_heuristic(Heuristic *heuristic,
  bool use_estimates,
  bool use_preferred_operators) {

    assert(use_estimates || use_preferred_operators);
    if (use_estimates || use_preferred_operators) {
        heuristics.push_back(heuristic);
    }
    if (use_estimates) {
        estimate_heuristics.push_back(heuristic);
        search_progress.add_heuristic(heuristic);
    }
    if(use_preferred_operators) {
        preferred_operator_heuristics.push_back(heuristic);
    }
}

void GeneralEagerBestFirstSearch::initialize() {
    g_learning_search_space = &search_space; //TODO:CR - check if we can get of this
    //TODO children classes should output which kind of search
    cout << "Conducting best first search" <<
        (reopen_closed_nodes? " with" : " without") << " reopening closes nodes" << endl;

    assert(open_list != NULL);
    assert(heuristics.size() > 0);

    for (unsigned int i = 0; i < heuristics.size(); i++)
        heuristics[i]->evaluate(*g_initial_state);
	open_list->evaluate(0, false);
    search_progress.inc_evaluated();

    if(open_list->is_dead_end()) {
        assert(open_list->dead_end_is_reliable());
        cout << "Initial state is a dead end." << endl;
    }
    else {
        search_progress.get_initial_h_values();
        if (f_evaluator) {
            f_evaluator->evaluate(0,false);
            search_progress.report_f_value(f_evaluator->get_value());
        }
        search_progress.check_h_progress(0);
        SearchNode node = search_space.get_node(*g_initial_state);
        node.open_initial(heuristics[0]->get_value());

        open_list->insert(node.get_state_buffer());
    }
}


void GeneralEagerBestFirstSearch::statistics() const {
    search_progress.print_statistics();
    search_space.statistics();
}

int GeneralEagerBestFirstSearch::step() {
    pair<SearchNode, bool> n = fetch_next_node();
    if (!n.second) {
        return FAILED;
    }
    SearchNode node = n.first;

    State s = node.get_state();
    if (check_goal_and_set_plan(s))
        return SOLVED;

    vector<const Operator *> applicable_ops;
    set<const Operator *> preferred_ops;

    g_successor_generator->generate_applicable_ops(s, applicable_ops);
    // This evaluates the expanded state (again) to get preferred ops
    for (int i = 0; i < preferred_operator_heuristics.size(); i++) {
        vector<const Operator *> pref;
        pref.clear();
        preferred_operator_heuristics[i]->evaluate(s);
        preferred_operator_heuristics[i]->get_preferred_operators(pref);
        for (int i = 0; i < pref.size(); i++) {
            preferred_ops.insert(pref[i]);
        }
    }

    for(int i = 0; i < applicable_ops.size(); i++) {
        const Operator *op = applicable_ops[i];
        State succ_state(s, *op);
        search_progress.inc_generated();
        bool is_preferred = (preferred_ops.find(op) != preferred_ops.end());

        // get rid of self loops
        if (succ_state == s) continue;

        SearchNode succ_node = search_space.get_node(succ_state);

        if(succ_node.is_dead_end()) {
            // Previously encountered dead end. Don't re-evaluate.
            continue;
        }

    	// update new path
        bool eval_change = false;
    	if (g_do_path_dependent_search || succ_node.is_new()) {
    		for (unsigned int i = 0; i < heuristics.size(); i++) {
    			eval_change = eval_change || heuristics[i]->reach_state(s, *op, succ_node.get_state());
    		}
    		if (eval_change) {
    			succ_node.mark_to_eval();
    		}
		}

        if(succ_node.is_new()) {
            // We have not seen this state before.
            // Evaluate and create a new node.
            for (unsigned int i = 0; i < heuristics.size(); i++) {
                heuristics[i]->evaluate(succ_state);
            }
            search_progress.inc_evaluated();

            open_list->evaluate(node.get_g() + op->get_cost(), is_preferred);
            bool dead_end = open_list->is_dead_end() && open_list->dead_end_is_reliable();
            if (dead_end) {
                succ_node.mark_as_dead_end();
                continue;
            }

            //TODO:CR - add an ID to each state, and then we can use a vector to save per-state information
            int succ_h = heuristics[0]->get_heuristic();
            succ_node.open(succ_h, node, op);

			open_list->insert(succ_node.get_state_buffer());
            search_progress.check_h_progress(succ_node.get_g());

        } else if(succ_node.get_g() > node.get_g() + op->get_cost()) {
            // We found a new cheapest path to an open or closed state.
            if (reopen_closed_nodes) {
            	//TODO:CR - test if we should add a reevaluate flag and if it helps
                // if we reopen closed nodes, do that
                if(succ_node.is_closed()) {
                    /* TODO: Verify that the heuristic is inconsistent.
                     * Otherwise, this is a bug. This is a serious
                     * assertion because it can show that a heuristic that
                     * was thought to be consistent isn't. Therefore, it
                     * should be present also in release builds, so don't
                     * use a plain assert. */
                	//TODO:CR - add a consistent flag to heuristics, and add an assert here based on it
                    search_progress.inc_reopened();
                }
                succ_node.reopen(node, op);
                heuristics[0]->set_evaluator_value(succ_node.get_h());
				open_list->evaluate(node.get_g() + op->get_cost(), is_preferred);

                open_list->insert(succ_node.get_state_buffer());
            }
            else {
                // if we do not reopen closed nodes, we just update the parent pointers
                // Note that this could cause an incompatibility between
                // the g-value and the actual path that is traced back
                succ_node.update_parent(node, op);
            }
        }
    }

    return IN_PROGRESS;
}

pair<SearchNode, bool> GeneralEagerBestFirstSearch::fetch_next_node() {
    while(true) {
        if(open_list->empty()) {
            cout << "Completely explored state space -- no solution!" << endl;
            return make_pair(search_space.get_node(*g_initial_state), false);
            //assert(false);
            //exit(1); // fix segfault in release mode
            // TODO: Deal with this properly. step() should return
            //       failure.
        }

        int f = -1;
        int h = -1;
        State state(open_list->remove_min());
        if (g_do_path_dependent_search) {
        	assert(open_list->get_last_key_removed().size() == 2);
        	f = open_list->get_last_key_removed()[0];
        	h = open_list->get_last_key_removed()[1];
        }

        SearchNode node = search_space.get_node(state);

        // If the node is closed, we do not reopen it, as our heuristic
        // is consistent.
        // TODO: check this
        if (!node.is_closed() &&
        	    (!g_do_path_dependent_search || (h == node.get_h()))) {
        	if (g_do_path_dependent_search && node.need_eval()) {
        		for (unsigned int i = 0; i < heuristics.size(); i++) {
					heuristics[i]->evaluate(node.get_state());
				}
				search_progress.inc_evaluated();

				open_list->evaluate(node.get_g(), true);
				bool dead_end = open_list->is_dead_end() && open_list->dead_end_is_reliable();
				if (dead_end) {
					node.mark_as_dead_end();
					continue;
				}
				int new_h = heuristics[0]->get_heuristic();
				if (new_h > node.get_h()) {
					assert(node.is_open());
					node.update_h(new_h);
					open_list->insert(node.get_state_buffer());
					continue;
				}
        	}
            node.close();
            assert(!node.is_dead_end());
			update_jump_statistic(node);
            search_progress.inc_expanded();
            return make_pair(node, true);
        }
    }
}

void GeneralEagerBestFirstSearch::dump_search_space()
{
  search_space.dump();
}

void GeneralEagerBestFirstSearch::update_jump_statistic(const SearchNode& node) {
	if (f_evaluator) {
		heuristics[0]->set_evaluator_value(node.get_h());
		f_evaluator->evaluate(node.get_g(), false);
		int new_f_value = f_evaluator->get_value();
		search_progress.report_f_value(new_f_value);
        for (int i = 0; i < heuristics.size(); i++) {
            heuristics[i]->report_f_value(this,new_f_value);
        }
    }
}

void GeneralEagerBestFirstSearch::print_heuristic_values(
        const vector<int>& values) const {
    for(int i = 0; i < values.size(); i++) {
        cout << values[i];
        if(i != values.size() - 1)
            cout << "/";
    }
}

void GeneralEagerBestFirstSearch::set_f_evaluator(ScalarEvaluator* eval) {
    f_evaluator = eval;
}

void GeneralEagerBestFirstSearch::set_open_list(OpenList<state_var_t *> *open) {
    open_list = open;
}
