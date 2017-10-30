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
	 dump_plans(opts.get<bool>("dump_plans")),
	 num_node_expansions(0),
	 djkstra_initialized(false){
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
						solution_found = true;
                        break;
					}
				}
                else {
					resume_astar();
				}
			}
			if (!open_list->empty() && queue_djkstra.empty()) {
				resume_astar();
			}

            if (open_list->empty()) {
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

	if (dump_plans) 
		output_plans();

	plan_reconstructor->save_plans(top_k_plans, dump_plans);
	cout << "Actual search time: " << timer
         << " [t=" << utils::g_timer << "]" << endl;
}

bool KStar::enough_nodes_expanded() {
	if (open_list->empty())
		return true;
	update_most_expensive_succ();
	int max_plan_cost =  optimal_solution_cost + most_expensive_successor;
	if (max_plan_cost <= next_node_f)
		return true;
	return false;
}

void KStar::resume_astar() {
	cout << "Resuming A*" << endl;
	interrupted = false;
}

void KStar::update_most_expensive_succ() {
    if(queue_djkstra.empty())
		return;
	Node n = queue_djkstra.top();
    most_expensive_successor = n.g + pg_succ_generator->get_max_successor_delta(n, pg_root);
}

void KStar::set_optimal_plan_cost() {
	optimal_solution_cost = calculate_plan_cost(top_k_plans[0]);
}

void KStar::initialize_djkstra() {
   	if(djkstra_initialized || goal_state == StateID::no_state)
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
	pg_root->id = g_djkstra_nodes;
	++g_djkstra_nodes;
	//notify_expand(*pg_root, &state_registry, num_node_expansions);
    plan_reconstructor->add_plan(*pg_root, top_k_plans, simple_plans_only);
	statistics.inc_plans_found();
    set_optimal_plan_cost();
    Node successor;
    pg_succ_generator->get_successor_pg_root(pg_root, successor);
    queue_djkstra.push(successor);
	statistics.inc_total_djkstra_generations();
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

void KStar::throw_everything() {
	djkstra_initialized = false;		
	top_k_plans.clear();
	num_node_expansions = 0;
	statistics.reset_plans_found();
	statistics.reset_opt_found();
	queue_djkstra = std::priority_queue<Node>();
}

// Djkstra search on path graph P(G) returns true if enough plans have been found
bool KStar::djkstra_search() {
	std::cout << "Switching to djkstra search on path graph" << std::endl;
	// When Djkstra restarts remove everything from its last iteration 
	throw_everything();
	statistics.inc_djkstra_runs();
	initialize_djkstra();
    while (!queue_djkstra.empty()) {
		Node node = queue_djkstra.top();
 		if(!enough_nodes_expanded())
			return false;
		queue_djkstra.pop();

		//notify_expand(node, &state_registry, num_node_expansions);
		plan_reconstructor->add_plan(node, top_k_plans, simple_plans_only);
		statistics.inc_plans_found();
		if (enough_plans_found())
			return true;

		init_tree_heaps(node);
		std::vector<Node> successors;
		pg_succ_generator->get_successors(node, successors);

        for (auto succ : successors) {
			queue_djkstra.push(succ);
			statistics.inc_total_djkstra_generations();
		}
	}
	return false;
}

void KStar::dump_path_graph() {
	std::stringstream stream, node_stream;	
	std::string filename = "path_graph.dot";
	stream << "digraph {\n" << endl ;
	PerStateInformation<SearchNodeInfo>& search_node_infos = 
			search_space.search_node_infos;
	for (PerStateInformation<SearchNodeInfo>::const_iterator it =\
		 search_node_infos.begin(&state_registry); 
		 it != search_node_infos.end(&state_registry); ++it) {	
		StateID id = *it;	
		GlobalState s = state_registry.lookup_state(id);
		begin_subgraph(s.get_state_tuple(), stream);
		for (size_t i = 0; i  < tree_heap[s].size(); ++i){
			Sap sap = tree_heap[s][i];
			std::string id = get_sap_id(sap, s);
			std::string label = get_sap_label(sap);	
			Node node(0, sap, s.get_id());
			vector<Node> successors;
			pg_succ_generator->get_successors(node, successors, true);
			for (auto& succ : successors) {
				GlobalState heap_state = 
						state_registry.lookup_state(succ.heap_state);
				std::string succ_id = get_sap_id(succ.sap, heap_state); 
				add_edge(id, succ_id, std::to_string(succ.g), stream);
			}	
		}			
		stream << "}" << endl ;
	}
	save_and_close(filename, stream, node_stream);
}

void KStar::dump_dot() const {
		std::stringstream stream, node_stream;
		stream << "digraph {\n";
		for (PerStateInformation<SearchNodeInfo>::const_iterator it =\
         search_space.search_node_infos.begin(&state_registry);
			 it != search_space.search_node_infos.end(&state_registry); ++it) {
			StateID id = *it;
			stream << id.hash() << " [ peripheries=\"1\", shape=\"rectangle\", ";
			GlobalState s = state_registry.lookup_state(id);
			const SearchNodeInfo &node_info = search_space.search_node_infos[s];

			if (test_goal(s)) {
				stream << "style=\"rounded, filled\", fillcolor=\"red\", ";
			}
			else {
				stream << "style=\"rounded, filled\", fillcolor=\"yellow\", ";
			}
			stream << "label=\"#"<< id.hash() << "\\n" << "s="<< state_label(s) << "\\n";
			stream << "\" ]\n";

			for (Sap sap: incomming_heap[s]) {
                node_stream << id.hash() << "  ->  " << sap->get_from_state().get_id().hash();
				node_stream <<" [ label=\"" << sap->op->get_name();
				node_stream << "/"<< sap->op->get_cost();
                node_stream << "\"";
				if (node_info.parent_state_id == sap->get_from_state().get_id()) {
					node_stream << " style=\"dashed\" color=\"#A9A9A9 \"";
				}
				node_stream << " ]\n";
			}
		}

		node_stream << "}\n";

		// Write state space to dot file
		std::ofstream file;
		file.open("full_state_space.dot", std::ofstream::out);
		file << stream.rdbuf();
		file << node_stream.rdbuf();
		file.close();
}

void KStar::dump_tree_edge() {
	std::stringstream stream;	
	std::string filename = "tree_edges";
	for (PerStateInformation<SearchNodeInfo>::const_iterator it =\
		 search_space.search_node_infos.begin(&state_registry); 
		 it != search_space.search_node_infos.end(&state_registry); ++it) {	
		StateID id = *it;	
		GlobalState s = state_registry.lookup_state(id);
        const SearchNodeInfo &node_info = search_space.search_node_infos[s];
		if (node_info.creating_operator != -1 
			&& node_info.parent_state_id != StateID::no_state) {
			stream << s.get_state_tuple();	
			stream << ": ";
			stream << g_operators[node_info.creating_operator].get_name(); 	
		}
		stream << ""<< endl;
	}	
	
	std::ofstream file;
	file.open(filename, std::ofstream::out);
	file << stream.rdbuf();
	file.close();
}


void add_simple_plans_only_option(OptionParser &parser) {
	parser.add_option<bool>("simple_plans_only", "", "false");
}


static SearchEngine *_parse(OptionParser &parser) {
	 parser.add_option<ScalarEvaluator *>("eval", "evaluator for h-value");
	 parser.add_option<int>("plans", "Number of plans", "1000");
	 parser.add_option<bool>("dump_plans", "Print plans", "false");

	 top_k_eager_search::add_pruning_option(parser);
	 add_simple_plans_only_option(parser);
	 SearchEngine::add_options_to_parser(parser);
	 Options opts = parser.parse();
	 KStar *engine = nullptr;
	 if (!parser.dry_run()) {
		 int num_plans = opts.get<int>("plans");
		 cout << "Running K* with K=" << num_plans << endl;
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
