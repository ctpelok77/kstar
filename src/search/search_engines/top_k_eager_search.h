#ifndef SEARCH_ENGINES_TOP_K_EAGER_SEARCH_H
#define SEARCH_ENGINES_TOP_K_EAGER_SEARCH_H

#include "../search_engine.h"
#include "../option_parser.h"
#include "../open_lists/open_list.h"

#include <memory>
#include <vector>

class GlobalOperator;
class Heuristic;
class PruningMethod;
class ScalarEvaluator;

namespace options {
class Options;
}

namespace top_k_eager_search {
class TopKEagerSearch : public SearchEngine {
    const bool reopen_closed_nodes;
    const int number_of_plans;
    std::unique_ptr<StateOpenList> open_list;
    ScalarEvaluator *f_evaluator;
    std::vector<Heuristic *> heuristics;
    std::vector<Heuristic *> preferred_operator_heuristics;

    std::shared_ptr<PruningMethod> pruning_method;
	bool interrupt_search;

    std::pair<SearchNode, bool> fetch_next_node();
    void start_f_value_statistics(EvaluationContext &eval_context);
    void update_f_value_statistics(const SearchNode &node);
    void reward_progress();
    void print_checkpoint_line(int g) const;
    virtual bool check_goal_and_get_plans(const GlobalState &state);

protected:
    virtual void initialize() override;
    virtual SearchStatus step() override;
	void interrupt();
	void resume();	
public:
    explicit TopKEagerSearch(const options::Options &opts);
    virtual ~TopKEagerSearch() = default;

    virtual void print_statistics() const override;

    void dump_search_space() const;
};

void add_top_k_option(OptionParser &parser);
void add_pruning_option(OptionParser &parser);
}

#endif
