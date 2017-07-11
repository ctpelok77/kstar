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
	:TopKEagerSearch(opts), first_plan_found(false) {
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
			
		// First solution found 
		if (status == SOLVED && !first_plan_found) {
			interrupt();		
			add_first_plan();
			add_goal_heap_top();
		}
		
		// Check whether A* has expanded enough nodes and if yes 
		// start a djkstra search on P(G)
		if (status == INTERRUPTED) {
			if (open_list->empty() && queue_djkstra.empty()) {
				output_plans();	
				return;
			}	
		
			if (open_list->empty()) {
				djkstra_search();
			}
		
			// g_n = g-value of the top node in Djkstras queue	
			// f_u = f-value of the top node in Astar queue
			int g_n = queue_djkstra.top().first;	
			int f_u = get_f_value(open_list->top());
			if (optimal_solution_cost + g_n <= f_u) {
				djkstra_search();				
			}	
			else {
				debug(4, _ARGS);
				// TODO: add functor struct
				//resume();
			}
		}
	}
		cout << "Actual search time: " << timer
         << " [t=" << utils::g_timer << "]" << endl;
}

int KStar::get_f_value(StateID id) {
	GlobalState s = state_registry.lookup_state(id);		
	int g = search_space.get_node(s).get_g();
	EvaluationContext eval_context(s, g, false, &statistics);
	int f = eval_context.get_heuristic_value(f_evaluator);
	return f;
}

void KStar::add_first_plan() {
	GlobalState s = state_registry.lookup_state(goal_state);
	Plan plan;	
	search_space.trace_path(s, plan, nullptr);
	optimal_solution_cost = calculate_plan_cost(plan); 
	top_k_plans.push_back(plan);		
	first_plan_found = true;
}


// add the top node from the goal heap to the open list  
void KStar::add_goal_heap_top() {
	GlobalState s =  state_registry.lookup_state(goal_state);
	queue_djkstra.push(0, H_in[s].top());	
}

// Djkstra search on path graph P(G)
void KStar::djkstra_search() {
	exit(-1);
    while (!queue_djkstra.empty()) {
		std::pair<int, StateActionPair> top_pair = queue_djkstra.pop();
        int old_f = top_pair.first;
		StateActionPair sap = top_pair.second; 
		GlobalState s = state_registry.lookup_state(sap.state_id);
        const int g = top_pair.first;   
        int new_f = g;
        if (new_f < old_f)
            continue;	

		// TODO: Termination criterion needed 
		// TODO: Check whether successor generation is correct,  i.e. agrees  
        while (!H_in[s].empty()) {
			GlobalOperator op = g_operators[sap.op_index];
			GlobalState succ_state = state_registry.get_successor_state(s, op); 
            StateActionPair succ_sap = H_in[succ_state].top(); 
			H_in[succ_state].pop();
			// TODO: make sure that the cost is correct here
            const int cost = 0; 
            int succ_g = g + cost;
            int f = succ_g;
            queue_djkstra.push(f, succ_sap);
        }
    }
}

void KStar::dump_astar_search_space() {
	search_space.dump_dot();
}

void KStar::dump_djkstra_search() {
			
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
