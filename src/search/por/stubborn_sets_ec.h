#ifndef POR_STUBBORN_SETS_EC_H
#define POR_STUBBORN_SETS_EC_H

#include "stubborn_sets.h"

namespace StubbornSetsEC {
struct StubbornDTG {
    struct Arc {
        Arc(int target_value_, int operator_no_)
            : target_value(target_value_),
              operator_no(operator_no_) {
        }
        int target_value;
        int operator_no;
    };

    struct Node {
        std::vector<Arc> outgoing;
        std::vector<Arc> incoming;
    };

    std::vector<Node> nodes;
    std::vector<bool> goal_values;

    void recurse_forwards(int value, std::vector<bool> &reachable) const;
    void recurse_backwards(int value, std::vector<bool> &relevant) const;
public:
    StubbornDTG() {}
    ~StubbornDTG() {}

    void forward_reachability_analysis(int start_value,
                                       std::vector<bool> &reachable) const;
    void backward_relevance_analysis(std::vector<bool> &relevant) const;
};

class StubbornSetsEC : public StubbornSets::StubbornSets {
private:
    std::vector<StubbornDTG> dtgs;
    std::vector<std::vector<std::vector<bool>>> reachability_map;
    std::vector<std::vector<int>> v_precond;
    std::vector<bool> active_ops;
    std::vector<std::vector<int>> conflicting_and_disabling;
    std::vector<std::vector<int>> disabled;
    std::vector<bool> written_vars;
    std::vector<std::vector<bool>> nes_computed;

    void get_disabled_vars(int op1_no, int op2_no, std::vector<int> &disabled_vars);
    void build_dtgs();
    void build_reachability_map();
    void compute_v_precond();
    void compute_conflicts_and_disabling();
    void compute_disabled_by_o();
    void add_conflicting_and_disabling(int op_no, const GlobalState &state);
    void recurse_forwards(int var, int start_value, int current_value, std::vector<bool> &reachable);
    void compute_active_operators(const GlobalState &state);
    void mark_as_stubborn(int op_no, const GlobalState &state);
    void add_nes_for_fact(std::pair<int, int> fact, const GlobalState &state);
    void apply_s5(const GlobalOperator &op, const GlobalState &state);
protected:
    virtual void initialize();
    virtual void do_pruning(const GlobalState &state, std::vector<const GlobalOperator *> &ops);
public:
    StubbornSetsEC();
    ~StubbornSetsEC();
    virtual void dump_options() const;
};
}
#endif
