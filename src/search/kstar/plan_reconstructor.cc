#include "plan_reconstructor.h"
#include "util.h"

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
	Node &current_sap = node;
    //cout << "djkstra traceback" << endl;
	//cout << "sigma = ";
	for(;;) {
		path.push_back(current_sap);
        if(!current_sap.sap->op) {
	//		cout << "R";
			break;
        }
        //cout << "("<< current_sap.sap->op->get_name() << ") " << flush;
		current_sap = parent_node.at(current_sap);
	}
	reverse(path.begin(), path.end());
    //cout << "" << endl;
    return path;
}

vector<Node> PlanReconstructor::compute_sidetrack_seq(vector<Node>& path) {
	//cout << "seq = ";
	vector<Node> seq;
	int last_index = path.size() - 1;
    Node last_element = path[last_index];
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
			current_state = state_registry->lookup_state(seq[seq_index].sap->from);
			++seq_index;
		}
		else {
			// prepend tree edge
			const GlobalOperator *op = &g_operators[info.creating_operator];
            plan.push_back(op);
            current_state = state_registry->lookup_state(info.parent_state_id);
		}
		state_seq.push_back(current_state.get_id());
	}
	reverse(plan.begin(), plan.end());
}

// Check whether the traceback of node yields a simple path in
// the path graph. This is a neccessary condition for the extracted
// plans to be simple
/*bool PlanReconstructor::is_simple_path(Node node) {
	std::unordered_set<Node> seen;
	vector<Node> path = djkstra_traceback(node);
    for (int i = 0; i < path.size(); ++i) {
        if (seen.find(path[i])  == seen.end()) {
			seen.insert(path[i]);
        }
        else {
           return false;
        }
    }
    return true;
}
*/

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
    if (simple_plans_only) {
		if (is_simple_plan(state_seq, state_registry)) {
			top_k_plans.push_back(plan);
			save_plan(plan, true);
		}
	}
	else {
		top_k_plans.push_back(plan);
        save_plan(plan, true);
	}
}
}
