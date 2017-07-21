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
	print_set_of_operators(g_operators, "all ops");
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
			search_space.dump_dot();
			add_first_plan();
		}
		
		// Check whether A* has expanded enough nodes and if yes 
		// start a djkstra search on P(G)
		if (status == INTERRUPTED) {
			djkstra_search();
			if (open_list->empty() && queue_djkstra.empty()) {
				return;
			}	
			
			if (open_list->empty()) {
				//djkstra_search();
			}
			// g_n = g-value of the top node in Djkstras queue	
			// f_u = f-value of the top node in Astar queue
			int g_n = queue_djkstra.top().first;	
			int f_u = get_f_value(open_list->top());
			if (optimal_solution_cost + g_n <= f_u) {
				//djkstra_search();				
			}	
			else {
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

// add R which is the node to start with in djkstra search 
void KStar::add_goal_heap_top() {
	GlobalState s =  state_registry.lookup_state(goal_state);
	StateActionPair r(StateID::no_state, 
					  H_in[s].top().to,
					  nullptr, nullptr, nullptr);
	queue_djkstra.push(std::pair<int, StateActionPair>(0, r));
}

std::string KStar::get_node_name(StateActionPair& p) {
	std::string str; 
	if (p.from == StateID::no_state) {
		return "R"; 
	}	

	GlobalState s = p.get_from_state();		
	GlobalState succ = p.get_to_state();
	str = "(" + std::to_string(s[0]) + "," + std::to_string(succ[0])  + ")";
	return str;	
}

void KStar::notify_generate(StateActionPair& p) {
	std::string node_name = get_node_name(p);	
	std::cout << "Generating node " << node_name << " from queue"<< std::endl;
}

void KStar::notify_push(StateActionPair& p) {
	std::string node_name = get_node_name(p);	
	std::cout << "Pushing node " << node_name << " to queue" << std::endl;
}

void KStar::notify_expand(StateActionPair& p) {
	std::string node_name = get_node_name(p);	
	std::cout << "Expanding node "<< node_name << std::endl;
}

// Djkstra search on path graph P(G)
void KStar::djkstra_search() {
	std::cout << "Switching to djkstra search on path graph" << std::endl;
	add_goal_heap_top();
    while (!queue_djkstra.empty()) {
		std::pair<int, StateActionPair> top_pair = queue_djkstra.top();
		queue_djkstra.top();
		notify_expand(top_pair.second);	
        int old_f = top_pair.first;
		StateActionPair sap = top_pair.second; 
        const int g = top_pair.first;   
        int new_f = g;
        if (new_f < old_f)
            continue;	
		
		GlobalState s = state_registry.lookup_state(sap.to);
        while (!H_T[s].empty()) {
            StateActionPair succ_sap = H_T[s].top();
			H_T[s].pop();
			notify_generate(succ_sap);
            int f = g;
           	if (sap.from != StateID::no_state) {
				f += get_cost_heap_edge(sap, succ_sap);
			}
            else {
				f += get_cost_cross_edge(succ_sap);
			}
            queue_djkstra.push(std::pair<int, StateActionPair>(f, succ_sap));
			notify_push(succ_sap);
        }
    }
}

void KStar::dump_astar_search_space() {
	search_space.dump_dot();
}

int KStar::get_cost_heap_edge(StateActionPair& from, StateActionPair& to) {
	int cost_heap_edge = to.get_delta() -from.get_delta();
	assert(cost_heap_edge >= 0);
	return cost_heap_edge;	
}

int KStar::get_cost_cross_edge(StateActionPair& to) {
	int cost_cross_edge = to.get_delta();		
	assert(cost_cross_edge >= 0);
	return cost_cross_edge;
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
