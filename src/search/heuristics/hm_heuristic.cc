#include "hm_heuristic.h"

#include "../option_parser.h"
#include "../plugin.h"
#include "../task_tools.h"

#include <cassert>
#include <limits>
#include <set>

using namespace std;

namespace hm_heuristic {
HMHeuristic::HMHeuristic(const Options &opts)
    : Heuristic(opts),
      m(opts.get<int>("m")),
      has_cond_effects(has_conditional_effects(task_proxy)) {
}


HMHeuristic::~HMHeuristic() {
}


bool HMHeuristic::dead_ends_are_reliable() const {
    return !has_axioms(task_proxy) && !has_cond_effects;
}


void HMHeuristic::initialize() {
    cout << "Using h^" << m << "." << endl;
    cout << "The implementation of the h^m heuristic is preliminary." << endl
         << "It is SLOOOOOOOOOOOW." << endl
         << "Please do not use this for comparison!" << endl;
    generate_all_tuples();
}


int HMHeuristic::compute_heuristic(const GlobalState &global_state) {
    State state = convert_global_state(global_state);
    if (is_goal_state(task_proxy, state)) {
        return 0;
    } else {
        Tuples s_tup;
        state_to_tuple(state, s_tup);

        init_hm_table(s_tup);
        update_hm_table();
        //dump_table();

        Tuples goals;
        for (FactProxy goal : task_proxy.get_goals()) {
            goals.emplace_back(goal.get_variable().get_id(), goal.get_value());
        }
        int h = eval(goals);

        if (h == numeric_limits<int>::max())
            return DEAD_END;
        return h;
    }
}


void HMHeuristic::init_hm_table(const Tuples &t) {
    for (auto &hm_ent : hm_table) {
        const Tuples &tup = hm_ent.first;
        int h_val = check_tuple_in_tuple(tup, t);
        hm_table[tup] = h_val;
    }
}


void HMHeuristic::update_hm_table() {
    int round = 0;
    do {
        ++round;
        was_updated = false;

        for (OperatorProxy op : task_proxy.get_operators()) {
            Tuples pre;
            get_operator_pre(op, pre);

            int c1 = eval(pre);
            if (c1 != numeric_limits<int>::max()) {
                Tuples eff;
                get_operator_eff(op, eff);
                vector<Tuples> partial_effs;
                generate_all_partial_tuples(eff, partial_effs);
                for (Tuples &partial_eff : partial_effs) {
                    update_hm_entry(partial_eff, c1 + op.get_cost());

                    int eff_size = partial_eff.size();
                    if (eff_size < m) {
                        extend_tuple(partial_eff, op);
                    }
                }
            }
        }
    } while (was_updated);
}


void HMHeuristic::extend_tuple(const Tuples &t, const OperatorProxy &op) {
    for (auto &hm_ent : hm_table) {
        const Tuples &tup = hm_ent.first;
        bool contradict = false;
        for (const FactPair &fact : tup) {
            if (contradict_effect_of(op, fact.var, fact.value)) {
                contradict = true;
                break;
            }
        }
        if (!contradict && (tup.size() > t.size()) && (check_tuple_in_tuple(t, tup) == 0)) {
            Tuples pre;
            get_operator_pre(op, pre);

            Tuples others;
            for (const FactPair &fact : tup) {
                if (find(t.begin(), t.end(), fact) == t.end()) {
                    others.push_back(fact);
                    if (find(pre.begin(), pre.end(), fact) == pre.end()) {
                        pre.push_back(fact);
                    }
                }
            }

            sort(pre.begin(), pre.end());


            set<int> vars;
            bool is_valid = true;
            for (FactPair &fact : pre) {
                if (vars.count(fact.var) != 0) {
                    is_valid = false;
                    break;
                }
                vars.insert(fact.var);
            }

            if (is_valid) {
                int c2 = eval(pre);
                if (c2 != numeric_limits<int>::max()) {
                    update_hm_entry(tup, c2 + op.get_cost());
                }
            }
        }
    }
}


int HMHeuristic::eval(const Tuples &t) const {
    vector<Tuples> partial;
    generate_all_partial_tuples(t, partial);
    int max = 0;
    for (Tuples &fact : partial) {
        assert(hm_table.count(fact) == 1);

        int h = hm_table.at(fact);
        if (h > max) {
            max = h;
        }
    }
    return max;
}


int HMHeuristic::update_hm_entry(const Tuples &t, int val) {
    assert(hm_table.count(t) == 1);
    if (hm_table[t] > val) {
        hm_table[t] = val;
        was_updated = true;
    }
    return val;
}


int HMHeuristic::check_tuple_in_tuple(
    const Tuples &tup, const Tuples &big_tuple) const {
    for (auto &fact0 : tup) {
        bool found = false;
        for (auto &fact1 : big_tuple) {
            if (fact0 == fact1) {
                found = true;
                break;
            }
        }
        if (!found) {
            return numeric_limits<int>::max();
        }
    }
    return 0;
}


void HMHeuristic::state_to_tuple(const State &state, Tuples &t) const {
    for (FactProxy fact : state) {
        t.emplace_back(fact.get_variable().get_id(), fact.get_value());
    }
}


void HMHeuristic::get_operator_pre(const OperatorProxy &op, Tuples &t) const {
    for (FactProxy pre : op.get_preconditions()) {
        t.emplace_back(pre.get_variable().get_id(), pre.get_value());
    }
    sort(t.begin(), t.end());
}


void HMHeuristic::get_operator_eff(const OperatorProxy &op, Tuples &t) const {
    for (EffectProxy eff : op.get_effects()) {
        FactProxy fact = eff.get_fact();
        t.emplace_back(fact.get_variable().get_id(), fact.get_value());
    }
    sort(t.begin(), t.end());
}


bool HMHeuristic::contradict_effect_of(
    const OperatorProxy &op, int var, int val) const {
    for (EffectProxy eff : op.get_effects()) {
        FactProxy fact = eff.get_fact();
        if (fact.get_variable().get_id() == var && fact.get_value() != val) {
            return true;
        }
    }
    return false;
}


void HMHeuristic::generate_all_tuples() {
    Tuples t;
    generate_all_tuples_aux(0, m, t);
}


void HMHeuristic::generate_all_tuples_aux(int var, int sz, const Tuples &base) {
    int num_variables = task_proxy.get_variables().size();
    for (int i = var; i < num_variables; ++i) {
        int domain_size = task_proxy.get_variables()[i].get_domain_size();
        for (int j = 0; j < domain_size; ++j) {
            Tuples tup(base);
            tup.emplace_back(i, j);
            hm_table[tup] = 0;
            if (sz > 1) {
                generate_all_tuples_aux(i + 1, sz - 1, tup);
            }
        }
    }
}


void HMHeuristic::generate_all_partial_tuples(
    const Tuples &base_tuple, vector<Tuples> &res) const {
    Tuples t;
    generate_all_partial_tuples_aux(base_tuple, t, 0, m, res);
}


void HMHeuristic::generate_all_partial_tuples_aux(
    const Tuples &base_tuple, const Tuples &t, int index, int sz, vector<Tuples> &res) const {
    if (sz == 1) {
        for (size_t i = index; i < base_tuple.size(); ++i) {
            Tuples tup(t);
            tup.push_back(base_tuple[i]);
            res.push_back(tup);
        }
    } else {
        for (size_t i = index; i < base_tuple.size(); ++i) {
            Tuples tup(t);
            tup.push_back(base_tuple[i]);
            res.push_back(tup);
            generate_all_partial_tuples_aux(base_tuple, tup, i + 1, sz - 1, res);
        }
    }
}


void HMHeuristic::dump_table() const {
    for (auto &hm_ent : hm_table) {
        cout << "h[";
        dump_tuple(hm_ent.first);
        cout << "] = " << hm_ent.second << endl;
    }
}


void HMHeuristic::dump_tuple(const Tuples &tup) const {
    cout << tup[0].var << "=" << tup[0].value;
    for (const FactPair &fact : tup)
        cout << "," << fact.var << "=" << fact.value;
}


static Heuristic *_parse(OptionParser &parser) {
    parser.document_synopsis("h^m heuristic", "");
    parser.document_language_support("action costs", "supported");
    parser.document_language_support("conditional effects", "ignored");
    parser.document_language_support("axioms", "ignored");
    parser.document_property("admissible",
                             "yes for tasks without conditional "
                             "effects or axioms");
    parser.document_property("consistent",
                             "yes for tasks without conditional "
                             "effects or axioms");

    parser.document_property("safe",
                             "yes for tasks without conditional "
                             "effects or axioms");
    parser.document_property("preferred operators", "no");

    parser.add_option<int>("m", "subset size", "2", Bounds("1", "infinity"));
    Heuristic::add_options_to_parser(parser);
    Options opts = parser.parse();
    if (parser.dry_run())
        return 0;
    else
        return new HMHeuristic(opts);
}

static Plugin<Heuristic> _plugin("hm", _parse);
}
