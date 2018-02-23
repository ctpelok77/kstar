#include "original_goals_task.h"

#include "../globals.h"

#include "../option_parser.h"
#include "../plugin.h"

using namespace std;

namespace extra_tasks {
OriginalGoalsTask::OriginalGoalsTask() : DelegatingTask(g_root_task()) {
    // Copying from parent
    initial_state_values = parent->get_initial_state_values();
    initial_state_values.pop_back();
    // TODO: Should we keep the goals explicitly?
}

int OriginalGoalsTask::get_num_goals() const {
    int goal_op_index = parent->get_num_operators() - 1;
    return parent->get_num_operator_preconditions(goal_op_index, false) - 1;
}

FactPair OriginalGoalsTask::get_goal_fact(int index) const {
    int goal_op_index = parent->get_num_operators() - 1;
    return parent->get_operator_precondition(goal_op_index, index, false);
}

int OriginalGoalsTask::get_num_variables() const {
    return parent->get_num_variables() - 1;
}

int OriginalGoalsTask::get_num_operators() const {
    return parent->get_num_operators() - 1;
}

int OriginalGoalsTask::get_num_operator_preconditions(int index, bool is_axiom) const {
    return parent->get_num_operator_preconditions(index, is_axiom) - 1;
}

std::vector<int> OriginalGoalsTask::get_initial_state_values() const {
    return initial_state_values;
}

void OriginalGoalsTask::convert_state_values_from_parent(std::vector<int> &values) const {
    values.pop_back();
}

static shared_ptr<AbstractTask> _parse(OptionParser &parser) {
    if (parser.dry_run())
        return nullptr;
    else
        return make_shared<OriginalGoalsTask>();
}

static PluginShared<AbstractTask> _plugin("original_goals", _parse);


}
