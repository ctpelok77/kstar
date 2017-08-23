#include "kstar.h"

#include "../plugin.h"
#include "../option_parser.h"
#include "../search_engines/search_common.h"
#include "../utils/util.h"
#include "../utils/countdown_timer.h"
#include "util.h"

using namespace top_k_eager_search;

namespace kstar{

KStar::KStar(const options::Options &opts)
	:TopKEagerSearch(opts),
	 simple_plans_only(opts.get<bool>("simple_plans_only")),
	 num_node_expansions(0), djkstra_initialized(false)
{
	pg_succ_generator =
			unique_ptr<SuccessorGenerator>(new SuccessorGenerator(
															tree_heap,
															incomming_heap,
															parent_node,
															cross_edge,
															&state_registry));
	plan_reconstructor =  unique_ptr<PlanReconstructor>(new PlanReconstructor(
													   parent_node,
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
			if (djkstra_search()) {
				status = SOLVED;
                solution_found = true;
			}
			interrupt();
		}

		// Check whether A* has expanded enough nodes and if yes
		// start a djkstra search on P(G)
		if (status == INTERRUPTED) {
			if (!open_list->empty() && !queue_djkstra.empty()) {
				// if enough nodes expanded do djkstra search
				if (enough_nodes_expanded()) {
					if (djkstra_search()) {
						status = SOLVED;
                        break;
					}
				}
                else {
					resume_astar();
				}
			}
            else if (open_list->empty()) {
				if (djkstra_search()) {
					status = SOLVED;
					solution_found = true;
					break;
				}
				else {
					status = FAILED;
					solution_found =  false;
					break;
				}
			}
		}
	}
    //output_plans();
	cout << "Actual search time: " << timer
         << " [t=" << utils::g_timer << "]" << endl;
}

bool KStar::enough_nodes_expanded() {
	if (open_list->empty()) 
		return true;	

	StateID u = open_list->top();
	Node n = queue_djkstra.top();
	int d = n.g + pg_succ_generator->get_max_successor_delta(n, pg_root);
	if (optimal_solution_cost + d <= get_f_value(u))
		return true;
	return false;
}

void KStar::resume_astar() {
	SearchControl sc;
    StateID u = open_list->top();
	Node n = queue_djkstra.top();
	sc.interrupt_immediatly = false;
	sc.f_u = get_f_value(u);
    sc.d = n.g + pg_succ_generator->get_max_successor_delta(n, pg_root);
	sc.optimal_solution_cost = optimal_solution_cost;
	resume(sc);
	cout << "Resuming A*" << endl;
}

void KStar::set_optimal_plan_cost() {
	optimal_solution_cost = calculate_plan_cost(top_k_plans[0]);
}

void KStar::initialize_djkstra() {
   	if(djkstra_initialized)
		return;

	GlobalState g = state_registry.lookup_state(goal_state);
    plan_reconstructor->set_goal_state(goal_state);
    init_tree_heap(g);
	if (tree_heap[g].empty())
       return;
	// Generate root of path graph
    Sap sap = make_shared<StateActionPair>(StateID::no_state,
                                           goal_state, nullptr,
                                           &state_registry, &search_space);
    pg_root = make_shared<Node>(0, sap, StateID::no_state);
	notify_expand(*pg_root, &state_registry, num_node_expansions);
    plan_reconstructor->add_plan(*pg_root, top_k_plans, simple_plans_only);
    set_optimal_plan_cost();
    Node successor;
    pg_succ_generator->get_successor_pg_root(pg_root, successor);
    queue_djkstra.push(successor);
    notify_push(successor, &state_registry);
    djkstra_initialized = true;
}

bool KStar::enough_plans_found() {
	int num_plans_found = top_k_plans.size();
	if (num_plans_found >= number_of_plans) {
		return true;
	}
	return false;
}

// init the neccessary tree heaps for the successor generation
// of s that is tree_heap[s] (obviously) and from (for the cross edge)
void KStar::init_tree_heaps(Node node) {
	GlobalState s = state_registry.lookup_state(node.heap_state);
	init_tree_heap(s);
    GlobalState from  = state_registry.lookup_state(node.sap->from);
	init_tree_heap(from);
}

// Djkstra search on path graph P(G) returns true if enough plans have been found
bool KStar::djkstra_search() {
	std::cout << "Switching to djkstra search on path graph" << std::endl;
	initialize_djkstra();
    while (!queue_djkstra.empty()) {
		Node node = queue_djkstra.top();
 		if(!enough_nodes_expanded())
			return false;

		notify_expand(node, &state_registry, num_node_expansions);
		plan_reconstructor->add_plan(node, top_k_plans, simple_plans_only);
		if (enough_plans_found())
			return true;

		queue_djkstra.pop();
		init_tree_heaps(node);
		std::vector<Node> successors;
		pg_succ_generator->get_successors(node, successors);

        for (auto succ : successors) {
			queue_djkstra.push(succ);
			notify_push(succ, &state_registry);
		}
	}
	return false;
}

void add_simple_plans_only_option(OptionParser &parser) {
    parser.add_option<bool>("simple_plans_only", "", "false");
}

 static SearchEngine *_parse(OptionParser &parser) {
	 parser.add_option<ScalarEvaluator *>("eval", "evaluator for h-value");
	 parser.add_option<int>("plans", "Number of plans", "5");

	 top_k_eager_search::add_pruning_option(parser);
	 add_simple_plans_only_option(parser);
	 SearchEngine::add_options_to_parser(parser);
	 Options opts = parser.parse();
	 KStar *engine = nullptr;
	 if (!parser.dry_run()) {
		 int num_plans = opts.get<int>("plans");
		 cout << "Running K* with K=" <<num_plans << endl;
		 auto temp = search_common::create_astar_open_list_factory_and_f_eval(opts);
		 opts.set("open", temp.first);
		 opts.set("f_eval", temp.second);
		 opts.set("reopen_closed", true);
		 std::vector<Heuristic *> preferred_list;
		 opts.set("preferred", preferred_list);
		 opts.set("K", num_plans);

		 engine = new KStar(opts);
	 }

    return engine;
}

static Plugin<SearchEngine> _plugin("kstar", _parse);
}
