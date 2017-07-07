#include "kstar.h"

#include "../plugin.h"
#include "../option_parser.h"
#include "../search_engines/search_common.h"
#include "../utils/util.h"
#include "../utils/countdown_timer.h"
#include "../utils/system.h"
#include "../utils/timer.h"

using namespace top_k_eager_search;

namespace kstar{
KStar::KStar(const options::Options &opts) 	
	:TopKEagerSearch(opts), first_solution_found(false) {
	//search_common::create_djkstra_search();
}

void KStar::search() {
	initialize();
	utils::CountdownTimer timer(max_time);
	while (status == IN_PROGRESS || status == INTERRUPTED) {
		status = step();
		if (timer.is_expired()) {
			cout << "Time limit reached. Abort search." << endl;
			status = TIMEOUT;
			break;
		}
			
		// Goal state has been reached for the first time	
		if (status == SOLVED && !first_solution_found) {
			interrupt();		
		}

		if (status == INTERRUPTED) {
			resume();
			djkstra_search();
		}
	}
		cout << "Actual search time: " << timer
         << " [t=" << utils::g_timer << "]" << endl;
}

// add all state action pairs (sap) that reached the goal state  	
void KStar::add_goal_sap() {
	GlobalState goal_s =  state_registry.lookup_state(goal_state);
	int size_goal_sap = H_in[goal_s].size();	
	for (int i = 0; i < size_goal_sap; ++i) {
				
	}	
}

void KStar::djkstra_search() {
	add_goal_sap();

    while (!queue_djkstra.empty()) {
		std::pair<int, StateActionPair> top_pair = queue_djkstra.pop();
        int old_f = top_pair.first;
		StateActionPair sap = top_pair.second; 
        const int g = top_pair.first;   
        int new_f = g;
        if (new_f < old_f)
            continue;	

		// TODO: Termination criterion here 
        //if (goals && goals->count(state) == 1) {
         //   return state;
        //}

        /*const Transitions &transitions = state->get_outgoing_transitions();
        for (const Transition &transition : transitions) {
            int op_id = transition.op_id;
            AbstractState *successor = transition.target;

            const int op_cost = operator_costs[op_id];
            int succ_g = (op_cost == INF) ? INF : g + op_cost;
            
			if (succ_g < successor->get_search_info().get_g_value()) {
                successor->get_search_info().decrease_g_value_to(succ_g);
                int f = succ_g;
                assert(f >= 0);
                queue_djkstra.push(f, successor);
                successor->get_search_info().set_incoming_transition(
                    Transition(op_id, state));
            }
        }
		*/
    }
}

static SearchEngine *_parse(OptionParser &parser) {
    parser.add_option<ScalarEvaluator *>("eval", "evaluator for h-value");

	top_k_eager_search::add_top_k_option(parser);
	top_k_eager_search::add_pruning_option(parser);
    SearchEngine::add_options_to_parser(parser);
    Options opts = parser.parse();

    KStar *engine = nullptr;
    if (!parser.dry_run()) {
        auto temp = search_common::create_astar_open_list_factory_and_f_eval(opts);
        opts.set("open", temp.first);
        opts.set("f_eval", temp.second);
        opts.set("reopen_closed", true);
		std::vector<Heuristic *> preferred_list;
        opts.set("preferred", preferred_list);
        engine = new KStar(opts);
    }

    return engine;
}
	
static Plugin<SearchEngine> _plugin("kstar", _parse);
}
