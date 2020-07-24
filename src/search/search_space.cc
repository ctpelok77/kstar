#include "search_space.h"

#include "global_operator.h"
#include "global_state.h"
#include "globals.h"
#include "successor_generator.h"

#include <cassert>
#include <sstream>
#include "search_node_info.h"
#include "utils/util.h"

using namespace std;

SearchNode::SearchNode(const StateRegistry &state_registry,
                       StateID state_id,
                       SearchNodeInfo &info,
                       OperatorCost cost_type)
    : state_registry(state_registry),
      state_id(state_id),
      info(info),
      cost_type(cost_type) {
    assert(state_id != StateID::no_state);
}

GlobalState SearchNode::get_state() const {
    return state_registry.lookup_state(state_id);
}

bool SearchNode::is_open() const {
    return info.status == SearchNodeInfo::OPEN;
}

bool SearchNode::is_closed() const {
    return info.status == SearchNodeInfo::CLOSED;
}

bool SearchNode::is_dead_end() const {
    return info.status == SearchNodeInfo::DEAD_END;
}

bool SearchNode::is_new() const {
    return info.status == SearchNodeInfo::NEW;
}

int SearchNode::get_g() const {
    assert(info.g >= 0);
    return info.g;
}

int SearchNode::get_real_g() const {
    return info.real_g;
}

void SearchNode::open_initial() {
    assert(info.status == SearchNodeInfo::NEW);
    info.status = SearchNodeInfo::OPEN;
    info.g = 0;
    info.real_g = 0;
    info.parent_state_id = StateID::no_state;
    info.creating_operator = -1;
}

void SearchNode::open(const SearchNode &parent_node,
                      const GlobalOperator *parent_op) {
    assert(info.status == SearchNodeInfo::NEW);
    info.status = SearchNodeInfo::OPEN;
    info.g = parent_node.info.g + get_adjusted_action_cost(*parent_op, cost_type);
    info.real_g = parent_node.info.real_g + parent_op->get_cost();
    info.parent_state_id = parent_node.get_state_id();
    info.creating_operator = get_op_index_hacked(parent_op);
}

void SearchNode::reopen(const SearchNode &parent_node,
                        const GlobalOperator *parent_op) {
    assert(info.status == SearchNodeInfo::OPEN ||
           info.status == SearchNodeInfo::CLOSED);

    // The latter possibility is for inconsistent heuristics, which
    // may require reopening closed nodes.
    info.status = SearchNodeInfo::OPEN;
    info.g = parent_node.info.g + get_adjusted_action_cost(*parent_op, cost_type);
    info.real_g = parent_node.info.real_g + parent_op->get_cost();
    info.parent_state_id = parent_node.get_state_id();
    info.creating_operator = get_op_index_hacked(parent_op);
}

// like reopen, except doesn't change status
void SearchNode::update_parent(const SearchNode &parent_node,
                               const GlobalOperator *parent_op) {
    assert(info.status == SearchNodeInfo::OPEN ||
           info.status == SearchNodeInfo::CLOSED);
    // The latter possibility is for inconsistent heuristics, which
    // may require reopening closed nodes.
    info.g = parent_node.info.g + get_adjusted_action_cost(*parent_op, cost_type);
    info.real_g = parent_node.info.real_g + parent_op->get_cost();
    info.parent_state_id = parent_node.get_state_id();
    info.creating_operator = get_op_index_hacked(parent_op);
}

void SearchNode::close() {
    assert(info.status == SearchNodeInfo::OPEN);
    info.status = SearchNodeInfo::CLOSED;
}

void SearchNode::unclose() {
    assert(info.status == SearchNodeInfo::CLOSED);
    info.status = SearchNodeInfo::OPEN;
}

void SearchNode::mark_as_dead_end() {
    info.status = SearchNodeInfo::DEAD_END;
}

void SearchNode::dump() const {
    cout << state_id << ": ";
    get_state().dump_fdr();
    if (info.creating_operator != -1) {
        cout << " created by " << g_operators[info.creating_operator].get_name()
             << " from " << info.parent_state_id;
    } else {
        cout << " no parent";
    }
    cout << ", g=" << get_real_g() << endl;
}

SearchSpace::SearchSpace(StateRegistry &state_registry, OperatorCost cost_type)
    : state_registry(state_registry),
      cost_type(cost_type),
	  plan_simulation_index(1) {
}

SearchNode SearchSpace::get_node(const GlobalState &state) {
    return SearchNode(
        state_registry, state.get_id(), search_node_infos[state], cost_type);
}

void SearchSpace::trace_path(const GlobalState &goal_state,
                             vector<const GlobalOperator *> &path) const {
    GlobalState current_state = goal_state;
    assert(path.empty());
    for (;;) {
        const SearchNodeInfo &info = search_node_infos[current_state];
        if (info.creating_operator == -1) {
            assert(info.parent_state_id == StateID::no_state);
            break;
        }
        assert(utils::in_bounds(info.creating_operator, g_operators));
        const GlobalOperator *op = &g_operators[info.creating_operator];
        path.push_back(op);
        current_state = state_registry.lookup_state(info.parent_state_id);
    }
    reverse(path.begin(), path.end());
}

void SearchSpace::dump_state(std::ostream& os, const GlobalState& state) const {
    int num_vars = state_registry.get_num_variables();
    vector<string> names;
    for (int var=0; var < num_vars; ++var) {
        string fact_name = state_registry.get_task().get_fact_name(FactPair(var, state[var]));        
        if (fact_name == "__special_value_false__" || fact_name == "__special_value_true__")
            continue;
        if (fact_name == "<none of those>")
            continue;
        if (fact_name.compare(0, 11, "NegatedAtom") == 0)
            continue;

        names.push_back(fact_name);
    }

    os << "[" << endl;
    size_t i = 0;
    for (; i < names.size() - 1; ++i) {
        os << "\"" << names[i] << "\"," << endl;
    }
    os << "\"" << names[i] << "\"" << endl;
    os << "]" << endl;
}

void SearchSpace::trace_from_plan(const std::vector<const GlobalOperator *> &plan, std::vector<StateID> &plan_trace) const {
    assert(plan_trace.size() == 0);
    GlobalState current_state = state_registry.get_initial_state();
    for (size_t i=0; i < plan.size(); ++i) {
        const GlobalOperator *op = plan[i];
        current_state = state_registry.get_successor_state(current_state, *op);
        plan_trace.push_back(current_state.get_id());
    }
}

void SearchSpace::dump_trace(const std::vector<StateID> &plan_trace, std::ostream& os) const {

    GlobalState current_state = state_registry.get_initial_state();
    dump_state(os, current_state);
    for (size_t i=0; i < plan_trace.size(); ++i) {
        os << "," << endl;
        dump_state(os, state_registry.lookup_state(plan_trace[i]));
    }
}

void SearchSpace::dump() const {
    for (PerStateInformation<SearchNodeInfo>::const_iterator it =
             search_node_infos.begin(&state_registry);
         it != search_node_infos.end(&state_registry); ++it) {
        StateID id = *it;
        GlobalState state = state_registry.lookup_state(id);
        const SearchNodeInfo &node_info = search_node_infos[state];
        cout << id << ": ";
        state.dump_fdr();
        if (node_info.creating_operator != -1 &&
            node_info.parent_state_id != StateID::no_state) {
            cout << " created by " << g_operators[node_info.creating_operator].get_name()
                 << " from " << node_info.parent_state_id << endl;
        } else {
            cout << "has no parent" << endl;
        }
    }
}

void SearchSpace::dump_dot() const {
	std::stringstream stream, node_stream;	
	stream << "digraph {\n";
    for (PerStateInformation<SearchNodeInfo>::const_iterator it =\
         search_node_infos.begin(&state_registry); 
		 it != search_node_infos.end(&state_registry); ++it) {   
		StateID id = *it;
		stream << id.hash() << " [ peripheries=\"1\", shape=\"rectangle\", ";				
        GlobalState s = state_registry.lookup_state(id);
        const SearchNodeInfo &node_info = search_node_infos[s];
		
		if (test_goal(s)) {
			stream << "style=\"rounded, filled\", fillcolor=\"red\", ";
		}
		else {
			stream << "style=\"rounded, filled\", fillcolor=\"yellow\", ";
		}
		stream << "label=\"#"<< id.hash() << "\\n" << "s="<< state_label(s) << "\\n";
		stream << "\" ]\n";

        if (node_info.creating_operator != -1 && node_info.parent_state_id != StateID::no_state) {
			node_stream << node_info.parent_state_id.hash() << "  ->  " << id.hash();
			node_stream <<" [ label=\"#" << g_operators[node_info.creating_operator].get_name();
			node_stream << "/"<< g_operators[node_info.creating_operator].get_cost();
			node_stream << "\"";
			if (g_operators[node_info.creating_operator].get_name() == "achieve_goal") {
				node_stream << " style=\"dashed\" color=\"#A9A9A9 \"";	
			}
			node_stream << " ]\n";
        } 		
    }

	node_stream << "}\n";
  
	// Write state space to dot file
	std::ofstream file;
	file.open("state_space.dot", std::ofstream::out);
	file << stream.rdbuf();
	file << node_stream.rdbuf();
	file.close();
}

void SearchSpace::print_statistics() const {
    cout << "Number of registered states: "
         << state_registry.size() << endl;
}

// This method can be used to see whether a path in the search space 
// is actually applicable (1), is within the cost bound (2) achieves the
// claimed utility and is within the claimed costs (3) 
void SearchSpace::simulate_path(
	vector<const GlobalOperator *> &path,  
    vector<StateID> &sequence,
    int claimed_cost) {
	
	if (path.empty()) 
		return;
	std::cout << "Simulate sas_plan." << plan_simulation_index << flush <<  std::endl;	
    GlobalState s = state_registry.lookup_state(sequence.front());
    int cost = 0;
    for (size_t i =  0; i < path.size(); ++i) {
        const GlobalOperator* op = path[i]; 
        GlobalState s = state_registry.lookup_state(sequence[i]);
        check_transition(s.get_id(), op);
        cost += op->get_cost(); 
    }
    GlobalState second_last = state_registry.lookup_state(sequence.back()); 
    GlobalState last_state = state_registry.get_successor_state(second_last, *path.back());
    
	check_cost(claimed_cost, cost);
    std::cout << "Path simulation successful." << flush << std::endl;
	++plan_simulation_index;
}


void SearchSpace::raise_simulation_error(std::string cause) {
    std::cerr << "Error while plan simulation!" << endl;
    std::cerr << "Reason: "<< cause << endl;
    utils::exit_with(utils::ExitCode::CRITICAL_ERROR);
}

void SearchSpace::check_transition(StateID current, 
                                   const GlobalOperator* op)
{
    GlobalState current_state = state_registry.lookup_state(current);
    GlobalState succ_state = state_registry.get_successor_state(current_state, *op);   
    std::string cause;
    if (!op->is_applicable(current_state)) {
        cause = "Inapplicable operator:" + op->get_name() +" in state " \
                +"#" + std::to_string(current.hash()); 
        raise_simulation_error(cause);
    }
}

void SearchSpace::check_cost(int claimed_cost, int actual_cost)  {
    std::string cause;
    if (claimed_cost != actual_cost) {
        cause = "Claimed and actual costs mismatch. Claimed: "+ to_string(claimed_cost) \
              + " Actual: "+ to_string(actual_cost);
        raise_simulation_error(cause);
    }
}
