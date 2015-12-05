#include "epsilon_greedy_open_list.h"

#include "../plugin.h"

#include <memory>

using namespace std;


EpsilonGreedyOpenListFactory::EpsilonGreedyOpenListFactory(
    const Options &options)
    : options(options) {
}

unique_ptr<StateOpenList>
EpsilonGreedyOpenListFactory::create_state_open_list() {
    return make_unique_ptr<EpsilonGreedyOpenList<StateOpenListEntry>>(options);
}

unique_ptr<EdgeOpenList>
EpsilonGreedyOpenListFactory::create_edge_open_list() {
    return make_unique_ptr<EpsilonGreedyOpenList<EdgeOpenListEntry>>(options);
}

static shared_ptr<OpenListFactory> _parse(OptionParser &parser) {
    parser.document_synopsis(
        "Epsilon-greedy open list",
        "Chooses an entry uniformly randomly with probability "
        "'epsilon', otherwise it returns the minimum entry. "
        "The algorithm is based on\n\n"
        " * Richard Valenzano, Nathan R. Sturtevant, "
        "Jonathan Schaeffer, and Fan Xie.<<BR>>\n"
        " [A Comparison of Knowledge-Based GBFS Enhancements and "
        "Knowledge-Free Exploration "
        "http://www.aaai.org/ocs/index.php/ICAPS/ICAPS14/paper/view/7943/8066]."
        "<<BR>>\n "
        "In //Proceedings of the Twenty-Fourth International "
        "Conference on Automated Planning and Scheduling (ICAPS "
        "2014)//, pp. 375-379. AAAI Press 2014.\n\n\n");
    parser.add_option<ScalarEvaluator *>("eval", "scalar evaluator");
    parser.add_option<bool>(
        "pref_only",
        "insert only nodes generated by preferred operators", "false");
    parser.add_option<double>(
        "epsilon",
        "probability for choosing the next entry randomly",
        "0.2",
        Bounds("0.0", "1.0"));

    Options opts = parser.parse();
    if (parser.dry_run()) {
        return nullptr;
    } else {
        return make_shared<EpsilonGreedyOpenListFactory>(opts);
    }
}


static PluginShared<OpenListFactory> _plugin("epsilon_greedy", _parse);
