#ifndef SEARCH_SPACE_H
#define SEARCH_SPACE_H

#include "global_state.h"
#include "operator_cost.h"
#include "per_state_information.h"
#include "search_node_info.h"

#include <vector>

class GlobalOperator;
class GlobalState;
class Group;

class SearchNode {
    const StateRegistry &state_registry;
    StateID state_id;
    SearchNodeInfo &info;
    OperatorCost cost_type;
public:
    SearchNode(const StateRegistry &state_registry,
               StateID state_id,
               SearchNodeInfo &info,
               OperatorCost cost_type);

    StateID get_state_id() const {
        return state_id;
    }
    GlobalState get_state() const;

    bool is_new() const;
    bool is_open() const;
    bool is_closed() const;
    bool is_dead_end() const;

    int get_g() const;
    int get_real_g() const;

    void open_initial();
    void open(const SearchNode &parent_node,
              const GlobalOperator *parent_op);
    void reopen(const SearchNode &parent_node,
                const GlobalOperator *parent_op);
    void update_parent(const SearchNode &parent_node,
                       const GlobalOperator *parent_op);
    void close();
    void mark_as_dead_end();

    void dump() const;
};


class SearchSpace {
public:
    PerStateInformation<SearchNodeInfo> search_node_infos;
    StateRegistry &state_registry;
    OperatorCost cost_type;
	int plan_simulation_index;

    void trace_path_with_symmetries(const GlobalState &goal_state,
                                    std::vector<const GlobalOperator *> &path,
                                    const std::shared_ptr<Group> &group) const;
    SearchSpace(StateRegistry &state_registry, OperatorCost cost_type);

    SearchNode get_node(const GlobalState &state);
    void trace_path(const GlobalState &goal_state,
                    std::vector<const GlobalOperator *> &path,
                    const std::shared_ptr<Group> &group = nullptr) const;
    void dump() const;
    void print_statistics() const;
	void dump_dot() const;
	void check_cost(int claimed_cost, int actual_cost);
	void check_transition(StateID current, const GlobalOperator* op);
	void raise_simulation_error(std::string cause);
	void simulate_path(
		std::vector<const GlobalOperator *> &path,  
		std::vector<StateID> &sequence,
		int claimed_cost);
};

#endif
