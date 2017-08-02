#include "kstar_util.h" 

namespace kstar {

PlanReconstructor::PlanReconstructor(std::map<Sap, Sap>& parent_sap, 
									  std::map<Sap, bool>& cross_edge,
									  StateID goal_state,
									  StateRegistry* state_registry,
									  SearchSpace* search_space) 
:parent_sap(parent_sap), cross_edge(cross_edge), goal_state(goal_state), 
 state_registry(state_registry), search_space(search_space)  {
		
}

std::vector<Sap> PlanReconstructor::djkstra_traceback(Node& top_pair)	{
	vector<Sap> path;
	Sap current_sap = top_pair.second;
	for(;;) {
		if(!current_sap)
			break;
		path.push_back(current_sap);
		current_sap = parent_sap[current_sap];
	}
	reverse(path.begin(), path.end());
    return path;
}

vector<Sap> PlanReconstructor::compute_sidetrack_seq(vector<Sap>& path) {

	vector<Sap> seq;
	int last_index = path.size() - 1;
    Sap last_element = path[last_index];
    seq.push_back(last_element);
	for (size_t i = last_index; i >= 2; --i) {
       if (cross_edge[path[i-1]]) {
		   seq.push_back(path[i - 1]);
	   }
	}
    reverse(seq.begin(), seq.end());
    return seq;
}

void PlanReconstructor::add_plan(Node& top_pair, 
								  std::vector<Plan>& top_k_plans)	
{
    vector<Sap> path = djkstra_traceback(top_pair);
	vector<Sap> seq = compute_sidetrack_seq(path);

	std::vector<const GlobalOperator*> plan;
	GlobalState current_state = state_registry->lookup_state(goal_state);
	size_t seq_index = 0;
	for(;;) {
		const SearchNodeInfo &info = search_space->search_node_infos[current_state];
		if (info.creating_operator == -1) {
			assert(info.parent_state_id == StateID::no_state);
			break;
		}

		if(seq_index <= seq.size() - 1 && seq[seq_index]->to == current_state.get_id()) {
			// prepend edge from seq
			plan.push_back(seq[seq_index]->op);
			current_state = state_registry->lookup_state(seq[seq_index]->from);
			++seq_index;
		}
		else {
			// prepend tree edge
			const GlobalOperator *op = &g_operators[info.creating_operator];
            plan.push_back(op);
            current_state = state_registry->lookup_state(info.parent_state_id);
		}
	}
	reverse(plan.begin(), plan.end());
	top_k_plans.push_back(plan);
}

SuccessorGenerator::SuccessorGenerator(PerStateInformation<InHeap> &H_T, 
					   				   std::map<Sap, Sap> &parent_sap,
									   std::map<Sap, bool> &cross_edge,
									   StateRegistry* state_registry)
:H_T(H_T), parent_sap(parent_sap), cross_edge(cross_edge), state_registry(state_registry)
{

}

void SuccessorGenerator::get_successors(Node& node,
										vector<Node> &successors,
										bool successors_only) {
	GlobalState s = state_registry->lookup_state(node.second->to);
	// get all cross edges
	add_cross_edge(node, successors, true);

	// get all successor of tree edges	
	while (!H_T[s].empty()) {
        auto succ_sap = H_T[s].top();
		H_T[s].pop();
        int f = node.first;
		f += get_cost_heap_edge(node.second, succ_sap);
        auto p =  std::pair<int, Sap>(f, succ_sap);
		successors.push_back(p);
        if (!successors_only) {
			parent_sap.insert(pair<Sap, Sap>(succ_sap, node.second));
        	cross_edge.insert(pair<Sap, bool>(succ_sap, false));
		}
    }
}


int SuccessorGenerator::get_max_successor_delta(Node& node) {
    GlobalState s = state_registry->lookup_state(node.second->to);
    int max = -1;
	vector<Node> successors;
	get_successors(node, successors);

	for(auto succ : successors) {
		if (succ.second->get_delta() > max) {
			max = succ.second->get_delta();
		}
	}
    H_T[s].reset();
    return max;
}

void SuccessorGenerator::get_successor_R(Node& node, Node &successor) {
	Sap sap = node.second; 
	GlobalState to_state = sap->get_to_state();
	auto succ_sap = H_T[to_state].top();
	H_T[to_state].pop();
	int f = node.first + succ_sap->get_delta();
	successor =  std::pair<int, Sap>(f, succ_sap);
    
	parent_sap.insert(std::pair<Sap, Sap>(succ_sap, sap));
	cross_edge.insert(std::pair<Sap, bool>(succ_sap, true));
}

int SuccessorGenerator::get_cost_heap_edge(Sap& from, Sap& to) {
	int cost_heap_edge = to->get_delta() - from->get_delta();
	return cost_heap_edge;
}

int SuccessorGenerator::get_cost_cross_edge(Sap& to) {
	int cost_cross_edge = to->get_delta();
	return cost_cross_edge;
}

void SuccessorGenerator::add_cross_edge(Node &p,
										vector<Node> &successors,
										bool successors_only) {
    GlobalState from =  p.second->get_from_state();
	if (H_T[from].empty())
        return;

	int new_f = p.first + p.second->get_delta();
	Sap sap =  H_T[from].top();
    auto new_p  = std::pair<int, Sap>(new_f, sap);
    successors.push_back(p);
    if(!successors_only) {
		parent_sap.insert(std::pair<Sap, Sap>(sap, p.second));
    	cross_edge.insert(std::pair<Sap, bool>(sap, true));
	}
}
}
