#include "plan_reconstructor.h"
#include "util.h"

#include <dirent.h>
#include <sys/stat.h>


namespace kstar {

PlanReconstructor::PlanReconstructor(std::unordered_map<Node, Node>& parent_sap,
									  std::unordered_set<Edge>& cross_edge,
									  StateID goal_state,
									  StateRegistry* state_registry,
									  SearchSpace* search_space) 
:parent_node(parent_sap),
 cross_edge(cross_edge),
 goal_state(goal_state),
 state_registry(state_registry),
 search_space(search_space)  {
	

}

void PlanReconstructor::set_goal_state(StateID goal_state) {
	this->goal_state = goal_state;
}

std::vector<Node> PlanReconstructor::djkstra_traceback(Node node)	{
	vector<Node> path;
	Node current_sap = node;
	for(;;) {
		path.push_back(current_sap);
        if(!current_sap.sap->op) {
			//cout << "R";
			break;
        }
		//cout << "("<< current_sap.sap->op->get_name() << ") " << flush;
		current_sap = parent_node.at(current_sap);
	}
	reverse(path.begin(), path.end());
    //cout << "" << endl;
    //print_node_sequence(path, "djkstra_traceback");
    return path;
}

vector<Node> PlanReconstructor::compute_sidetrack_seq(vector<Node>& path) {
	//cout << "seq = ";
	vector<Node> seq;
	int last_index = path.size() - 1;
    Node last_element = path[last_index];
	//std::cout <<  path[last_index].sap->op->get_name() << std::endl;
    seq.push_back(last_element);

	for (size_t i = last_index; i >= 2; --i) {
       Edge e(path[i-1], path[i]);

		if(cross_edge.find(e) != cross_edge.end()) {
			//cout << path[i - 1].sap->op->get_name();
			seq.push_back(path[i - 1]);
		}
	}
    reverse(seq.begin(), seq.end());
	//cout << "" << endl;
	//print_node_sequence(seq, "seq");
    return seq;

}

void PlanReconstructor::extract_plan(vector<Node>& seq,
									Plan &plan,
                                    StateSequence &state_seq) {

	GlobalState current_state = state_registry->lookup_state(goal_state);
	state_seq.push_back(current_state.get_id());
	int seq_index = 0;
   	int seq_size = seq.size();
	for(;;) {
		const SearchNodeInfo &info = search_space->search_node_infos[current_state];
		// Initial state reached and all edges of seq consumed
		if (info.creating_operator == -1 && seq_index == seq_size) {
			assert(info.parent_state_id == StateID::no_state);
			break;
		}

		// second last edge in seq and attachable to what we already have
		if(seq_index <= seq_size - 1 && seq[seq_index].sap->to == current_state.get_id()) {
			// prepend edge from seq
			plan.push_back(seq[seq_index].sap->op);
			/*std::cout << "prepend seq " << seq[seq_index].sap->op->get_name();
			std::cout << " to " << seq[seq_index].sap->get_to_state().get_state_tuple();
			std::cout << " from " << seq[seq_index].sap->get_from_state().get_state_tuple();
			std::cout << ""<< endl;*/
			current_state = state_registry->lookup_state(seq[seq_index].sap->from);
			++seq_index;
		}
		else {
			// prepend tree edge
			const GlobalOperator *op = &g_operators[info.creating_operator];
            plan.push_back(op);
			/*std::cout << "prepend tree " << op->get_name();
			std::cout << " to " << current_state.get_state_tuple();
			std::cout << " from " << state_registry->lookup_state(info.parent_state_id).get_state_tuple();
			std::cout << ""<< endl;*/
            current_state = state_registry->lookup_state(info.parent_state_id);
		}
		state_seq.push_back(current_state.get_id());
	}
	reverse(plan.begin(), plan.end());
	//print_operator_sequence(plan, "plan");
}

bool PlanReconstructor::is_simple_plan(StateSequence seq, StateRegistry* state_registry) {
    PerStateInformation<bool> seen;
    for (size_t i = 0; i < seq.size(); ++i) {
        GlobalState s = state_registry->lookup_state(seq[i]);
        if (!seen[s]) {
           seen[s] = true;
        }
        else {
           return false;
        }
    }
    return true;
}

void PlanReconstructor::inc_optimal_plans(Plan &plan) {
	int cost = calculate_plan_cost(plan);

	if (g_num_optimal_plans == 0) {
		++g_num_optimal_plans; 
		g_optimal_cost = cost;
		return;
	}	

	if (cost == g_optimal_cost) {
		++g_num_optimal_plans;	
	}
					
}

void PlanReconstructor::add_plan(Node node,
								 std::vector<Plan>& top_k_plans,
								 bool simple_plans_only) {
    vector<Node> path = djkstra_traceback(node);
    vector<Node> seq;

	if (path.size() > 1) {
		 seq = compute_sidetrack_seq(path);
	}

	Plan plan;
	StateSequence state_seq;
    extract_plan(seq, plan, state_seq);
	plan.shrink_to_fit();
	plan.pop_back();

    if (simple_plans_only) {
		if (is_simple_plan(state_seq, state_registry)) {
			inc_optimal_plans(plan);
			top_k_plans.push_back(plan);
		}
	}
	else {
		inc_optimal_plans(plan);
		top_k_plans.push_back(plan);
	}
}

void PlanReconstructor::save_plans(std::vector<Plan>& top_k_plans) {
	for (auto& plan : top_k_plans) {
        save_plan(plan, true);
	}
}
}
