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
    bool dump_plans;
    Verbosity verbosity;
    typedef std::unordered_set<Plan> PlansSet;

    // Keep plans (sorted vectors of operators) in a set
    PlansSet accepted_plans;
    int attempted_plans;
    int last_plan_cost;
    int best_plan_cost;
    std::unordered_map<int, PlansSet> kept_plans; // Plans kept in a set by cost
    int number_of_kept_plans;

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

    bool keep_plan(const Plan& plan, int cost);
    void output_plan(const Plan& plan, int cost);
    void dump_plan_json(Plan plan, std::ostream& os, bool dump_states) const;
    void check_set_best_plan(int cost);

public:
    PlanReconstructor(std::unordered_map<Node, Node>& parent_sap,
                       std::unordered_set<Edge>& cross_edge,
                       StateID goal_state,
                       StateRegistry* state_registry,
                       SearchSpace* search_space,
                       bool skip_reorderings,
                       bool dump_plans,
                       Verbosity verbosity);

    virtual ~PlanReconstructor() = default;
    std::vector<Node> djkstra_traceback(Node node);
    std::vector<Node> compute_sidetrack_seq(std::vector<Node>& path);
    void extract_plan(vector<Node>& seq, Plan &plan, StateSequence &state_seq);
    bool is_simple_plan(StateSequence seq, StateRegistry* state_registry);
    void set_goal_state(StateID goal_state);
    bool add_plan(Node node, bool simple_plans_only);
    void dump_dot_plan(const Plan& plan);
    void clear();
    int get_last_added_plan_cost() const;
    void add_plan_explicit_no_check(Plan plan);

    void dump_plans_json(std::ostream& os, bool dump_states) const;
    size_t number_of_plans_found() const {return number_of_kept_plans; }

    // Run at the end of the successful Dijkstra iteration
    void write_non_optimal_plans();

};
}

#endif
