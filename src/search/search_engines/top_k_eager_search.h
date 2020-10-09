#ifndef SEARCH_ENGINES_TOP_K_EAGER_SEARCH_H
#define SEARCH_ENGINES_TOP_K_EAGER_SEARCH_H

#include "../search_engine.h"
#include "../option_parser.h"
#include "../open_lists/open_list.h"
#include "../state_action_pair.h"
#include "../utils/util.h"

#include "../kstar/kstar_types.h"

#include <memory>
#include <algorithm>
#include <vector>
#include <queue>
#include <iostream>

class GlobalOperator;
class Heuristic;
class PruningMethod;
class ScalarEvaluator;
namespace options {
class Options;
}

namespace top_k_eager_search {
typedef shared_ptr<StateActionPair> Sap;

class TopKEagerSearch : public SearchEngine {
    const bool reopen_closed_nodes;
protected:
    const int number_of_plans;
    const double quality_bound;
    std::unique_ptr<StateOpenList> open_list;
    ScalarEvaluator *f_evaluator;
    std::vector<Heuristic *> heuristics;
    std::vector<Heuristic *> preferred_operator_heuristics;
    std::shared_ptr<PruningMethod> pruning_method;
    bool interrupted;
    StateID goal_state = StateID::no_state;
    bool all_nodes_expanded = false;
    int counter = 0;
    // std::vector<kstar::StateSequence> top_k_plans_states;
    PerStateInformation<vector<Sap>> incomming_heap;
    PerStateInformation<vector<Sap>> tree_heap;

    // g-value of the most expensive successor of the current
    // top node of the djkstra queue
    int most_expensive_successor;
    // The f-value of the top node of the open list of A*
    int next_node_f;
    bool first_plan_found;
    kstar::Verbosity verbosity;

    // void update_next_node_f();
    // int get_f_value(StateID id);
    std::pair<SearchNode, bool> fetch_next_node();
    void start_f_value_statistics(EvaluationContext &eval_context);
    void update_f_value_statistics(const SearchNode &node);
    void reward_progress();
    void print_checkpoint_line(int g) const;
    virtual void initialize() override;
    virtual SearchStatus step() override;

    void interrupt();
    void add_incoming_edge(SearchNode node, const GlobalOperator *op,
                             SearchNode succ_node);
    void dump_incoming_heap(const GlobalState& s) const;
    void dump_tree_heap(const GlobalState& s) const;

    void remove_tree_edge(GlobalState s);
    void sort_and_remove(GlobalState  s);
    std::string get_node_label(StateActionPair &edge);
    std::string get_node_name(StateActionPair &edge);

public:
    explicit TopKEagerSearch(const options::Options &opts);
    virtual ~TopKEagerSearch() = default;
    virtual void print_statistics() const override;
    void init_tree_heap(GlobalState& state);
};

void add_top_k_option(OptionParser &parser);
void add_pruning_option(OptionParser &parser);
}

#endif
