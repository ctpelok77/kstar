#ifndef KSTAR_KSTAR_TYPES_H
#define KSTAR_KSTAR_TYPES_H

#include <memory>
#include "../state_action_pair.h"
#include "unordered_set"

using namespace std;

namespace kstar {
    typedef shared_ptr<StateActionPair> Sap;
    typedef std::pair<Node, Node> Edge;
    typedef std::vector<const GlobalOperator*> Plan;
    typedef std::vector<StateID> StateSequence;
    typedef std::stringstream Stream;
    typedef std::unordered_set<const GlobalOperator*> OperatorSet;

    enum class Verbosity {
        SILENT,
        NORMAL,
        VERBOSE
    };

}

#endif
