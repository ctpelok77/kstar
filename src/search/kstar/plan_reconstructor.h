#ifndef KSTAR_PLAN_RECONSTRUCTOR_H
#define KSTAR_PLAN_RECONSTRUCTOR_H

#include "kstar_types.h"

namespace kstar {

class PlanReconstructor {
    std::unordered_map<Node, Node> &parent_node;
    std::unordered_set<Edge> &cross_edge;
    StateID goal_state;
    StateRegistry* state_registry;
    SearchSpace* search_space;
    const bool skip_reorderings;
    Verbosity verbosity;
    typedef std::unordered_set<Plan> PlansSet;

    // Keep plans (sorted vectors of operators) in a set
    PlansSet accepted_plans;
    int attempted_plans;

    std::string fact_to_pddl(std::string fact) const;
    std::string restructure_fact(std::string fact) const;
    void dump_state_with_actions(const StateID& state, const OperatorSet& actions, std::ostream& os);
    void dump_action_json(const GlobalOperator *op, std::ostream& os);
    void dump_state_json(const StateID& state, std::ostream& os);
    void action_name_parsing(std::string op_name, std::vector<std::string>& parsed);
    bool is_duplicate(const Plan& plan);

    size_t get_hash_value(const Plan &plan) const {
        std::size_t seed = plan.size();
        for (auto &op : plan) {
            seed ^= op->get_index() + 0x9e3779b9 + (seed << 6) + (seed >> 2);
        }
        return seed;
    }

public:
    PlanReconstructor(std::unordered_map<Node, Node>& parent_sap,
                       std::unordered_set<Edge>& cross_edge,
                       StateID goal_state,
                       StateRegistry* state_registry,
                       SearchSpace* search_space,
                       bool skip_reorderings,
                       Verbosity verbosity);

    virtual ~PlanReconstructor() = default;
    std::vector<Node> djkstra_traceback(Node node);
    std::vector<Node> compute_sidetrack_seq(std::vector<Node>& path);
    void extract_plan(vector<Node>& seq, Plan &plan, StateSequence &state_seq);
    bool is_simple_plan(StateSequence seq, StateRegistry* state_registry);
    void set_goal_state(StateID goal_state);
    bool add_plan(Node node, std::vector<Plan>& plans, bool simple_plans_only);
    void save_plans(std::vector<Plan>& plans, bool dump_plans);
    void dump_dot_plan(const Plan& plan);
    void clear();
};
}

#endif
