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

void KStar::init_plan_reconstructor() {
	plan_reconstructor = unique_ptr<PlanReconstructor>(new PlanReconstructor(
													   parent_sap, 
													   cross_edge, 
													   goal_state, 
													   &state_registry,
													   &search_space)); 		
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
			
		// First solution found. Add R to path graph 
		if (status == SOLVED && !first_plan_found) {
			interrupt();		
			add_first_plan();
			init_plan_reconstructor();
			// remove that part later 
			djkstra_search();
			output_plans();
			exit(0);
		}
		
		// Check whether A* has expanded enough nodes and if yes 
		// start a djkstra search on P(G)
		if (status == INTERRUPTED) {
			if (!open_list->empty()) {
				if (!queue_djkstra.empty()) { 	
					StateID u = open_list->top();					
					Node top_pair = queue_djkstra.top(); 
				}
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
	init_tree_heap(s);
	set_initialized(s);
    if (H_T[s].empty()) {
		return;
	}
	auto r = shared_ptr<StateActionPair>(new StateActionPair(StateID::no_state, H_T[s].top()->to,
					                                         nullptr, &state_registry, &search_space));
	queue_djkstra.push(Node(0, r));
	parent_sap.insert(std::pair<Sap,Sap>(r, nullptr));
}

std::string KStar::get_node_name(Node& p) {
	std::string str; 
	if (p.second->from == StateID::no_state) {
		return "R"; 
	}	

	GlobalState s = p.second->get_from_state();
	GlobalState succ = p.second->get_to_state();
	str = "(" + std::to_string(s.get_id().get_value()) + ","\
           + std::to_string(succ.get_id().get_value())  + ")";
	return str;	
}

void KStar::notify_generate(Node& p) {
	std::string node_name = get_node_name(p);	
	std::cout << "Generating node " << node_name << " from queue"<< std::endl;
}

void KStar::notify_push(Node& p) {
	std::string node_name = get_node_name(p);	
	std::cout << "Pushing node " << node_name << " to queue" << std::endl;
}

void KStar::notify_expand(Node& p) {
	std::string node_name = get_node_name(p);
	std::cout << "Expanding node "<< node_name << p.first << std::endl;
}

void KStar::reset_initialized()	{
    std::unordered_set<StateID> empty;
    heap_initialized.swap(empty);
}

bool KStar::is_initialized(GlobalState &s){
	return heap_initialized.find(s.get_id()) != heap_initialized.end();
}

void KStar::set_initialized(GlobalState &s) {
	heap_initialized.insert(s.get_id());
}

void KStar::add_cross_edges(Node& p) {
    GlobalState from =  p.second->get_from_state();
    if(!is_initialized(from)){
		init_tree_heap(from);
		set_initialized(from);
	}

	if (H_T[from].empty())
        return;

	int new_f = p.first + p.second->get_delta();
	Sap sap =  H_T[from].top();
    auto new_p  = std::pair<int, Sap>(new_f, sap);
	queue_djkstra.push(new_p);
	parent_sap.insert(std::pair<Sap, Sap>(sap, p.second));
    cross_edge.insert(std::pair<Sap, bool>(sap, true));
}

bool KStar::enough_plans_found() {
    int num_plans_found = top_k_plans.size();
	if (num_plans_found >= number_of_plans) {
		return true;
	}
	return false;
}

// Djkstra search on path graph P(G)
void KStar::djkstra_search() {
	std::cout << "Switching to djkstra search on path graph" << std::endl;
	reset_initialized();
	add_goal_heap_top();
    while (!queue_djkstra.empty()) {
		std::pair<int, Sap> top_pair = queue_djkstra.top();
		queue_djkstra.pop();

		if (closed.find(*top_pair.second) != closed.end())
			continue;
		closed.insert(*top_pair.second);

		notify_expand(top_pair);
        int old_f = top_pair.first;
		auto sap = top_pair.second;
        const int g = top_pair.first;   
        int new_f = g;
        if (new_f < old_f)
            continue;	
		
		GlobalState s = state_registry.lookup_state(sap->to);
		if (!is_initialized(s)) {
			init_tree_heap(s);
			set_initialized(s);
		}

		if (sap->from == StateID::no_state) {
			GlobalState to_state = sap->get_to_state();
            auto succ_sap = H_T[to_state].top();
			H_T[to_state].pop();
			int f = top_pair.first + succ_sap->get_delta();
			auto p =  std::pair<int, Sap>(f, succ_sap);
            queue_djkstra.push(p);
			parent_sap.insert(std::pair<Sap, Sap>(succ_sap, sap));
			cross_edge.insert(std::pair<Sap, bool>(succ_sap, true));
			continue;
		}
		add_cross_edges(top_pair);
		plan_reconstructor->add_plan(top_pair, top_k_plans);
        if (enough_plans_found())
			return;

        while (!H_T[s].empty()) {
            auto succ_sap = H_T[s].top();
			H_T[s].pop();
            int f = g;
			f += get_cost_heap_edge(sap, succ_sap);
            auto p =  std::pair<int, Sap>(f, succ_sap);
            queue_djkstra.push(p);
            parent_sap.insert(std::pair<Sap, Sap>(succ_sap, sap));
            cross_edge.insert(std::pair<Sap, bool>(succ_sap, false));
        }
    }
}

int KStar::get_cost_heap_edge(Sap& from, Sap& to) {
	int cost_heap_edge = to->get_delta() - from->get_delta();
	return cost_heap_edge;
}

int KStar::get_cost_cross_edge(Sap& to) {
	int cost_cross_edge = to->get_delta();
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
