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
	:TopKEagerSearch(opts) {
	print_set_of_operators(g_operators, "all ops");	
	// TODO: think of a way how those unique ptr stuff can be made shorter
	pg_succ_generator = 
			unique_ptr<SuccessorGenerator>(new kstar::SuccessorGenerator(	
															H_T,
															parent_sap,
															cross_edge,
															&state_registry));
	graphviz_writer = 
			unique_ptr<graphviz_writer::GraphvizWriter>(new graphviz_writer::GraphvizWriter(&state_registry,&search_space));
			


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
	while (status == IN_PROGRESS || status == INTERRUPTED
		   || status == FIRST_PLAN_FOUND)
	{
		status = step();
		if (timer.is_expired()) {
			cout << "Time limit reached. Abort search." << endl;
			status = TIMEOUT;
			break;
		}
		// First solution found. Add R to path graph, perfom djkstra
		if (status == FIRST_PLAN_FOUND) {
			interrupt();
			//add_first_plan();
			init_plan_reconstructor();
			if (djkstra_search()) {
				status = SOLVED;
                solution_found = true;
			}

			output_plans();
			search_space.dump_dot();
			graphviz_writer->dump_inheap(H_in);
			exit(0);
		}
		// Check whether A* has expanded enough nodes and if yes
		// start a djkstra search on P(G)
		if (status == INTERRUPTED) {
			if (!open_list->empty() && !queue_djkstra.empty()) {
				// if enough nodes expanded do djkstra search
				if (enough_nodes_expanded()) {
					if (djkstra_search()) {
						status = SOLVED;
                        output_plans();
					}
				}
			}
            else if(!open_list->empty()) {
				resume_astar();
			}
		}
		// Both queues are empty == less than k solutions found
		//if(open_list->empty() && queue_djkstra.empty()){
        //   status = FAILED;
		//}
	}
		cout << "Actual search time: " << timer
         << " [t=" << utils::g_timer << "]" << endl;
}

bool KStar::enough_nodes_expanded() {
	StateID u = open_list->top();
	Node n = queue_djkstra.top();
	int d = pg_succ_generator->get_max_successor_delta(n);

	if (optimal_solution_cost + d <= get_f_value(u))
		return true;
	return false;
}
// TODO: look what is the problem here 
void KStar::resume_astar() {
	SearchControl sc;
    StateID u = open_list->top();
	Node n = queue_djkstra.top();
	int d = pg_succ_generator->get_max_successor_delta(n);
	(void) d;
	sc.interrupt_immediatly = false;
	sc.f_u = get_f_value(u);
    sc.d = pg_succ_generator->get_max_successor_delta(n);
	sc.optimal_solution_cost = optimal_solution_cost;
	resume(sc);
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

bool KStar::enough_plans_found() {
    int num_plans_found = top_k_plans.size();
	if (num_plans_found >= number_of_plans) {
		return true;
	}
	return false;
}

void KStar::initialize_tree_heaps(Sap& sap) {
	GlobalState s = state_registry.lookup_state(sap->to);
	if (!is_initialized(s)) {
		init_tree_heap(s);
		set_initialized(s);
	}
    if(sap->from == StateID::no_state)
		return;

	GlobalState from =  sap->get_from_state();
    if(!is_initialized(from)){
		init_tree_heap(from);
		set_initialized(from);
	}
}

// Djkstra search on path graph P(G)
bool KStar::djkstra_search() {
	std::cout << "Switching to djkstra search on path graph" << std::endl;
	reset_initialized();
	add_goal_heap_top();

    while (!queue_djkstra.empty()) {
		Node node = queue_djkstra.top();
		queue_djkstra.pop();

		if (closed.find(*node.second) != closed.end())
			continue;
		closed.insert(*node.second);

		notify_expand(node);
		auto sap = node.second;
		initialize_tree_heaps(sap);

		// handle root node R
		if (sap->from == StateID::no_state) {
            Node successor;
			pg_succ_generator->get_successor_R(node, successor);
			queue_djkstra.push(successor);
			continue;
		}

		// For all other nodes
		plan_reconstructor->add_plan(node, top_k_plans);
        if (enough_plans_found())
			return true;

		std::vector<Node> successors; 
		pg_succ_generator->get_successors(node, successors);
		for (auto& succ : successors) 
			queue_djkstra.push(succ);
    }

	return false;
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
