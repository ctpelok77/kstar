#include "plan_forbid_reformulated_task.h"

#include <cassert>
#include <sstream>

#include "../utils/system.h"

using namespace std;


namespace extra_tasks {
PlanForbidReformulatedTask::PlanForbidReformulatedTask(
    const shared_ptr<AbstractTask> parent,
	std::vector<int>&& plan)
    : DelegatingTask(parent),
	  forbidding_plan(move(plan)),
	  operators_on_plan(0) {
	// Compute the indexes for new operators
	// First go all operators that are not on the plan
	// Then go all ops from plan with index 1, then 2, then 3.
	vector<bool> on_plan;
	on_plan.assign(parent->get_num_operators(), false);
	for (int op_no : forbidding_plan) {
		if (!on_plan[op_no]) {
			operators_on_plan++;
			on_plan[op_no] = true;
		}
	}
	for (size_t i = 0; i < on_plan.size(); ++i) {
		if (!on_plan[i])
			reformulated_operator_indexes.push_back(i);
	}

	// Creating initial state values by copying from the parent and pushing the new variables initial values
	initial_state_values = parent->get_initial_state_values();
	initial_state_values.push_back(1);
	initial_state_values.push_back(1);
	initial_state_values.insert(initial_state_values.end(), forbidding_plan.size(), 0);
}

int PlanForbidReformulatedTask::get_num_non_plan_operators() const {
	return reformulated_operator_indexes.size();
}


int PlanForbidReformulatedTask::get_parent_op_index(int index) const {
	int p_index = get_plan_op_index(index);
	if (p_index < 0)
		return reformulated_operator_indexes[index];

	return forbidding_plan[p_index];
}

int PlanForbidReformulatedTask::get_op_type(int index) const {
	if (index < get_num_non_plan_operators())
		return 0;

	return ((index - get_num_non_plan_operators()) / forbidding_plan.size()) + 1;
}

int PlanForbidReformulatedTask::get_plan_op_index(int index) const {
	// Returns the index on the plan
	if (index < get_num_non_plan_operators())
		return -1;

	return (index - get_num_non_plan_operators()) % forbidding_plan.size();
}

int PlanForbidReformulatedTask::get_possible_var_index() const {
	return parent->get_num_variables();
}

int PlanForbidReformulatedTask::get_following_var_index(int op_index) const {
	return parent->get_num_variables() + 1 + get_plan_op_index(op_index);
}

int PlanForbidReformulatedTask::get_num_variables() const {
    return parent->get_num_variables() + forbidding_plan.size() + 2 ;
}

string PlanForbidReformulatedTask::get_variable_name(int var) const {
	if (var < parent->get_num_variables())
		return parent->get_variable_name(var);
	if (var == parent->get_num_variables())
		return "possible";
	int ind = var - parent->get_num_variables() - 1;
	return ("following" + static_cast<ostringstream*>( &(ostringstream() << ind ) )->str());
}

int PlanForbidReformulatedTask::get_variable_domain_size(int var) const {
	if (var < parent->get_num_variables())
		return parent->get_variable_domain_size(var);

	return 2;
}

int PlanForbidReformulatedTask::get_variable_axiom_layer(int var) const {
	if (var < parent->get_num_variables())
		return parent->get_variable_axiom_layer(var);

	return -1;
}

int PlanForbidReformulatedTask::get_variable_default_axiom_value(int var) const {
	if (var < parent->get_num_variables())
		return parent->get_variable_default_axiom_value(var);
	if (var <= parent->get_num_variables() + 1)
		return 1;
	return 0;
}

string PlanForbidReformulatedTask::get_fact_name(const FactPair &fact) const {
	if (fact.var < parent->get_num_variables())
		return parent->get_fact_name(fact);
	assert(fact.value >= 0 && fact.value <= 1);
	if (fact.value == 0)
		return "false";
	return "true";
}

bool PlanForbidReformulatedTask::are_facts_mutex(const FactPair &fact1, const FactPair &fact2) const {
	if (fact1.var < parent->get_num_variables() && fact2.var < parent->get_num_variables())
		return parent->are_facts_mutex(fact1, fact2);

	if (fact1.var >= parent->get_num_variables() &&
			fact1.var == fact2.var &&
			fact1.value != fact2.value)
		return true;

	return false;
//    ABORT("PlanForbidReformulatedTask doesn't support querying mutexes.");
}

int PlanForbidReformulatedTask::get_operator_cost(int index, bool is_axiom) const {
	if (is_axiom)
		return parent->get_operator_cost(index, is_axiom);

	int parent_op_index = get_parent_op_index(index);
	return parent->get_operator_cost(parent_op_index, is_axiom);
}

string PlanForbidReformulatedTask::get_operator_name(int index, bool is_axiom) const {
	if (is_axiom)
		return parent->get_operator_name(index, is_axiom);

	int parent_op_index = get_parent_op_index(index);
	string name = parent->get_operator_name(parent_op_index, is_axiom);
	int op_type = get_op_type(index);

	return (name + " " + static_cast<ostringstream*>( &(ostringstream() << op_type ) )->str());
}

int PlanForbidReformulatedTask::get_num_operators() const {
    return get_num_non_plan_operators() + 3 * forbidding_plan.size();
}

int PlanForbidReformulatedTask::get_num_operator_preconditions(int index, bool is_axiom) const {
	if (is_axiom)
		return parent->get_num_operator_preconditions(index, is_axiom);

	int op_type = get_op_type(index);
	assert(op_type >= 0 && op_type <= 3);

	int parent_op_index = get_parent_op_index(index);
	if (op_type == 0)
		return parent->get_num_operator_preconditions(parent_op_index, is_axiom);

	if (op_type == 1)
		return parent->get_num_operator_preconditions(parent_op_index, is_axiom) + 1;

	assert(op_type == 2 || op_type == 3);
	return parent->get_num_operator_preconditions(parent_op_index, is_axiom) + 2;
}

FactPair PlanForbidReformulatedTask::get_operator_precondition(
    int op_index, int fact_index, bool is_axiom) const {

	if (is_axiom)
		return parent->get_operator_precondition(op_index, fact_index, is_axiom);

	int op_type = get_op_type(op_index);
	assert(op_type >= 0 && op_type <= 3);

	int parent_op_index = get_parent_op_index(op_index);
	int parent_num_pre = parent->get_num_operator_preconditions(parent_op_index, is_axiom);
	if (fact_index < parent_num_pre)
		return parent->get_operator_precondition(parent_op_index, fact_index, is_axiom);

	if (op_type == 1)
		return FactPair(get_possible_var_index(), 0);

	// op_type == 2 || op_type == 3
	// first additional precondition is the same for both cases
	if (fact_index == parent_num_pre)
		return FactPair(get_possible_var_index(), 1);

	// second additional precondition
	if (op_type == 2)
		return FactPair(get_following_var_index(op_index), 0);

	assert(op_type == 3);
	return FactPair(get_following_var_index(op_index), 1);
}

int PlanForbidReformulatedTask::get_num_operator_effects(int op_index, bool is_axiom) const {
	if (is_axiom)
		return parent->get_num_operator_effects(op_index, is_axiom);

	int op_type = get_op_type(op_index);
	assert(op_type >= 0 && op_type <= 3);
	int parent_op_index = get_parent_op_index(op_index);
	if (op_type == 0 || op_type == 2)
		return parent->get_num_operator_effects(parent_op_index, is_axiom) + 1;

	if (op_type == 1)
		return parent->get_num_operator_effects(parent_op_index, is_axiom);

	assert(op_type == 3);
	return parent->get_num_operator_effects(parent_op_index, is_axiom) + 2;
}

int PlanForbidReformulatedTask::get_num_operator_effect_conditions(
    int op_index, int eff_index, bool is_axiom) const {
	if (is_axiom)
		return parent->get_num_operator_effect_conditions(op_index, eff_index, is_axiom);

	int parent_op_index = get_parent_op_index(op_index);
	int parent_num_effs = parent->get_num_operator_effects(parent_op_index, is_axiom);
	if (eff_index < parent_num_effs)
		return parent->get_num_operator_effect_conditions(parent_op_index, eff_index, is_axiom);

	// The additional effects are unconditional
	return 0;
}

FactPair PlanForbidReformulatedTask::get_operator_effect_condition(
    int op_index, int eff_index, int cond_index, bool is_axiom) const {
	int parent_op_index = op_index;
	if (!is_axiom)
		parent_op_index = get_parent_op_index(op_index);
	return parent->get_operator_effect_condition(parent_op_index, eff_index, cond_index, is_axiom);
}

FactPair PlanForbidReformulatedTask::get_operator_effect(
    int op_index, int eff_index, bool is_axiom) const {
	if (is_axiom)
		return parent->get_operator_effect(op_index, eff_index, is_axiom);

	int op_type = get_op_type(op_index);
	assert(op_type >= 0 && op_type <= 3);
	int parent_op_index = get_parent_op_index(op_index);
	int parent_num_effs = parent->get_num_operator_effects(parent_op_index, is_axiom);
	if (eff_index < parent_num_effs)
		return parent->get_operator_precondition(parent_op_index, eff_index, is_axiom);

	if (op_type == 0 || op_type == 2)
		return FactPair(get_possible_var_index(), 0);

	assert(op_type == 3);

	if (eff_index == parent_num_effs)
		return FactPair(get_following_var_index(op_index), 0);
	return FactPair(get_following_var_index(op_index) + 1, 1);
}

const GlobalOperator *PlanForbidReformulatedTask::get_global_operator(int , bool ) const {
	ABORT("PlanForbidReformulatedTask doesn't support getting the operator explicitly.");
}

int PlanForbidReformulatedTask::get_num_goals() const {
    return parent->get_num_goals() + 1;
}

FactPair PlanForbidReformulatedTask::get_goal_fact(int index) const {
	if (index < parent->get_num_goals())
		return parent->get_goal_fact(index);

    return FactPair(get_possible_var_index(), 0);
}

vector<int> PlanForbidReformulatedTask::get_initial_state_values() const {
    return initial_state_values;
}

void PlanForbidReformulatedTask::convert_state_values_from_parent(
    vector<int> &) const {
	ABORT("PlanForbidReformulatedTask doesn't support getting a state from the parent state.");
}

}
