#ifndef TASKS_ORIGINAL_GOALS_TASK_H
#define TASKS_ORIGINAL_GOALS_TASK_H

#include "delegating_task.h"

#include <vector>

namespace extra_tasks {
class OriginalGoalsTask : public tasks::DelegatingTask {
    std::vector<int> initial_state_values;

public:
    OriginalGoalsTask();
    ~OriginalGoalsTask() = default;

    virtual int get_num_goals() const override;
    virtual FactPair get_goal_fact(int index) const override;

    virtual int get_num_variables() const override;
    virtual int get_num_operators() const override;
    virtual int get_num_operator_preconditions(int index, bool is_axiom) const override;

    virtual std::vector<int> get_initial_state_values() const override;
    virtual void convert_state_values_from_parent(
        std::vector<int> &values) const override;


};
}

#endif
