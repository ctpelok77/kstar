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
            output_plans();
			exit(0);
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

	auto r = shared_ptr<StateActionPair>(new StateActionPair(StateID::no_state,
					  H_in[s].top()->to,
					  nullptr, &state_registry, &search_space));
	queue_djkstra.push(std::pair<int, s_StateActionPair>(0, r));
	parent_sap.insert(std::pair<s_StateActionPair,s_StateActionPair>(r, nullptr));
}

std::string KStar::get_node_name(std::pair<int, s_StateActionPair>& p) {
	std::string str; 
	if (p.second->from == StateID::no_state) {
		return "R"; 
	}	

	GlobalState s = p.second->get_from_state();
	GlobalState succ = p.second->get_to_state();
	str = "(" + std::to_string(s[0]) + "," + std::to_string(succ[0])  + ")";
	return str;	
}

void KStar::notify_generate(std::pair<int, s_StateActionPair>& p) {
	std::string node_name = get_node_name(p);	
	std::cout << "Generating node " << node_name << " from queue"<< std::endl;
}

void KStar::notify_push(std::pair<int, s_StateActionPair>& p) {
	std::string node_name = get_node_name(p);	
	std::cout << "Pushing node " << node_name << " to queue" << std::endl;
}

void KStar::notify_expand(std::pair<int, s_StateActionPair>& p) {
	std::string node_name = get_node_name(p);
	std::cout << "Expanding node "<< node_name << " " << p.first << std::endl;
}

void KStar::add_cross_edges(std::pair<int, s_StateActionPair> p) {
    GlobalState from =  p.second->get_from_state();
    if(!heap_initialized[from]) {
		init_tree_heap(from);
        heap_initialized[from] = true;
	}

	if (H_T[from].empty())
        return;

	int new_f = p.first + p.second->get_delta();
	s_StateActionPair sap =  H_T[from].top();
    auto new_p  = std::pair<int, s_StateActionPair>(new_f, sap);
	queue_djkstra.push(new_p);
	parent_sap.insert(std::pair<s_StateActionPair, s_StateActionPair>(sap, p.second));
    cross_edge.insert(std::pair<s_StateActionPair, bool>(sap, true));
	notify_push(new_p);
}

// Djkstra search on path graph P(G)
void KStar::djkstra_search() {
	std::cout << "Switching to djkstra search on path graph" << std::endl;
	add_goal_heap_top();
    while (!queue_djkstra.empty()) {
		std::pair<int, s_StateActionPair> top_pair = queue_djkstra.top();
		queue_djkstra.pop();
		notify_expand(top_pair);
        int old_f = top_pair.first;
		auto sap = top_pair.second;
        const int g = top_pair.first;   
        int new_f = g;
        if (new_f < old_f)
            continue;	
		
		GlobalState s = state_registry.lookup_state(sap->to);
		if (!heap_initialized[s]) {
			init_tree_heap(s);
			//TODO: reset this for each Djkstra run
            heap_initialized[s] = true;
		}

		if (sap->from == StateID::no_state) {
			GlobalState to_state = sap->get_to_state();
            auto succ_sap = H_T[to_state].top();
			H_T[to_state].pop();
			int f = top_pair.first + succ_sap->get_delta();
			auto p =  std::pair<int, s_StateActionPair>(f, succ_sap);
            queue_djkstra.push(p);
			parent_sap.insert(std::pair<s_StateActionPair, s_StateActionPair>(succ_sap, sap));
			cross_edge.insert(std::pair<s_StateActionPair, bool>(succ_sap, true));
			continue;
		}
		add_cross_edges(top_pair);
		add_plan(top_pair);
        while (!H_T[s].empty()) {
            auto succ_sap = H_T[s].top();
			H_T[s].pop();
            int f = g;
			f += get_cost_heap_edge(sap, succ_sap);
            auto p =  std::pair<int, s_StateActionPair>(f, succ_sap);
            queue_djkstra.push(p);
            parent_sap.insert(std::pair<s_StateActionPair, s_StateActionPair>(succ_sap, sap));
            cross_edge.insert(std::pair<s_StateActionPair, bool>(succ_sap, false));
			notify_push(p);
        }
    }
}

vector<s_StateActionPair> KStar::djkstra_traceback(std::pair<int, s_StateActionPair>& top_pair)	{
	vector<s_StateActionPair> path;
	s_StateActionPair current_sap = top_pair.second;
	for(;;) {
		if(!current_sap)
			break;
		path.push_back(current_sap);
		current_sap = parent_sap[current_sap];
	}
	reverse(path.begin(), path.end());
    return path;
}

vector<s_StateActionPair> KStar::compute_sidetrack_seq(std::pair<int, s_StateActionPair>& top_pair,
													   vector<s_StateActionPair>& path) {

	vector<s_StateActionPair> seq;
	(void) top_pair;
	int last_index = path.size() - 1;
    s_StateActionPair last_element = path[last_index];
    seq.push_back(last_element);
	for (size_t i = last_index; i >= 2; --i) {
       if (cross_edge[path[i-1]])
			seq.push_back(path[i-1]);
	}
    return seq;
}

void KStar::add_plan(std::pair<int, s_StateActionPair>& top_pair)	{
    vector<s_StateActionPair> path = djkstra_traceback(top_pair);
	vector<s_StateActionPair> seq = compute_sidetrack_seq(top_pair, path);

	std::vector<const GlobalOperator*> plan;
	GlobalState current_state = state_registry.lookup_state(goal_state);
	size_t seq_index = 0;
	for(;;) {
		cout << "current state: " << current_state[0] << endl;
		const SearchNodeInfo &info = search_space.search_node_infos[current_state];
		if (info.creating_operator == -1) {
			assert(info.parent_state_id == StateID::no_state);
			break;
		}

		if(seq_index <= seq.size() - 1 && seq[seq_index]->to == current_state.get_id()) {
			// prepend edge from seq
			plan.push_back(seq[seq_index]->op);
			current_state = state_registry.lookup_state(seq[seq_index]->from);
			++seq_index;
		}
		else {
			// prepend tree edge
			const GlobalOperator *op = &g_operators[info.creating_operator];
            plan.push_back(op);
            current_state = state_registry.lookup_state(info.parent_state_id);
		}
	}
	reverse(plan.begin(), plan.end());
	top_k_plans.push_back(plan);
}

void KStar::dump_astar_search_space() {
	search_space.dump_dot();
}

int KStar::get_cost_heap_edge(s_StateActionPair& from, s_StateActionPair& to) {
	int cost_heap_edge = to->get_delta() - from->get_delta();
	assert(cost_heap_edge >= 0);
	return cost_heap_edge;	
}

int KStar::get_cost_cross_edge(s_StateActionPair& to) {
	int cost_cross_edge = to->get_delta();
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
