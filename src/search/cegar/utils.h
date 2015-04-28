#ifndef CEGAR_UTILS_H
#define CEGAR_UTILS_H

#include <limits>
#include <memory>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

class AbstractState;
class AbstractTask;
class AdditiveHeuristic;
class FactProxy;
class GoalsProxy;
class OperatorProxy;
class State;
class TaskProxy;

namespace cegar {
extern bool DEBUG;

const int UNDEFINED = -1;
// The name INFINITY is reserved in C++11.
const int INF = std::numeric_limits<int>::max();

// See additive_heuristic.h.
const int MAX_COST_VALUE = 100000000;

using Fact = std::pair<int, int>;
using VarToValues = std::unordered_map<int, std::vector<int> >;

struct Split {
    const int var_id;
    const std::vector<int> values;

    Split(int var_id, std::vector<int> &&values)
        : var_id(var_id), values(values) {
    }
};

using Splits = std::vector<Split>;

std::shared_ptr<AdditiveHeuristic> get_additive_heuristic(std::shared_ptr<AbstractTask> task);

/*
  The set of relaxed-reachable facts is the possibly-before set of facts that
  can be reached in the delete-relaxation before 'fact' is reached the first
  time plus 'fact' itself.
*/
std::unordered_set<FactProxy> get_relaxed_reachable_facts(TaskProxy task, FactProxy fact);

int get_pre(OperatorProxy op, int var_id);
int get_eff(OperatorProxy op, int var_id);
int get_post(OperatorProxy op, int var_id);

void reserve_memory_padding();
void release_memory_padding();
bool memory_padding_is_reserved();
}

#endif
