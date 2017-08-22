#include "eager_search.h"

#include "search_common.h"

#include "../evaluation_context.h"
#include "../globals.h"
#include "../heuristic.h"
#include "../option_parser.h"
#include "../plugin.h"
#include "../pruning_method.h"
#include "../successor_generator.h"

#include "../algorithms/ordered_set.h"

#include "../open_lists/open_list_factory.h"

#include "../structural_symmetries/group.h"

#include <cassert>
#include <cstdlib>
#include <memory>
#include <set>

using namespace std;

namespace eager_search {
EagerSearch::EagerSearch(const Options &opts)
    : SearchEngine(opts),
      reopen_closed_nodes(opts.get<bool>("reopen_closed")),
      use_multi_path_dependence(opts.get<bool>("mpd")),
      open_list(opts.get<shared_ptr<OpenListFactory>>("open")->
                create_state_open_list()),
      f_evaluator(opts.get<ScalarEvaluator *>("f_eval", nullptr)),
      preferred_operator_heuristics(opts.get_list<Heuristic *>("preferred")),
      pruning_method(opts.get<shared_ptr<PruningMethod>>("pruning")),
	  dump_forbid_plan_reformulation(opts.get<bool>("dump_forbid_plan_reformulation")) {
    if (opts.contains("symmetries")) {
        group = opts.get<shared_ptr<Group>>("symmetries");
        if (group && !group->is_initialized()) {
            cout << "Initializing symmetries (eager search)" << endl;
            group->compute_symmetries();
        }

        if (use_dks()) {
            cout << "Setting group in registry for DKS search" << endl;
            state_registry.set_group(group);
        }
    } else {
        group = nullptr;
    }
}

bool EagerSearch::use_oss() const {
    return group && group->has_symmetries() && group->get_search_symmetries() == SearchSymmetries::OSS;
}

bool EagerSearch::use_dks() const {
    return group && group->has_symmetries() && group->get_search_symmetries() == SearchSymmetries::DKS;
}

void EagerSearch::initialize() {
    cout << "Conducting best first search"
         << (reopen_closed_nodes ? " with" : " without")
         << " reopening closed nodes, (real) bound = " << bound
         << endl;
    if (use_multi_path_dependence)
        cout << "Using multi-path dependence (LM-A*)" << endl;
    assert(open_list);

    set<Heuristic *> hset;
    open_list->get_involved_heuristics(hset);

    // Add heuristics that are used for preferred operators (in case they are
    // not also used in the open list).
    hset.insert(preferred_operator_heuristics.begin(),
                preferred_operator_heuristics.end());

    // Add heuristics that are used in the f_evaluator. They are usually also
    // used in the open list and are hence already included, but we want to be
    // sure.
    if (f_evaluator) {
        f_evaluator->get_involved_heuristics(hset);
    }

    heuristics.assign(hset.begin(), hset.end());
    assert(!heuristics.empty());

    if (use_oss() || use_dks()) {
        assert(heuristics.size() == 1);
    }
    // Changed to copy the state to be able to reassign it.
    GlobalState initial_state = state_registry.get_initial_state();
    if (use_oss()) {
        int *canonical_state = group->get_canonical_representative(initial_state);
        initial_state = state_registry.register_state_buffer(canonical_state);
        delete canonical_state;
    }
    for (Heuristic *heuristic : heuristics) {
        heuristic->notify_initial_state(initial_state);
    }

    // Note: we consider the initial state as reached by a preferred
    // operator.
    EvaluationContext eval_context(initial_state, 0, true, &statistics);

    statistics.inc_evaluated_states();

    if (open_list->is_dead_end(eval_context)) {
        cout << "Initial state is a dead end." << endl;
    } else {
        if (search_progress.check_progress(eval_context))
            print_checkpoint_line(0);
        start_f_value_statistics(eval_context);
        SearchNode node = search_space.get_node(initial_state);
        node.open_initial();

        open_list->insert(eval_context, initial_state.get_id());
    }

    print_initial_h_values(eval_context);

    pruning_method->initialize(g_root_task());
}

void EagerSearch::print_checkpoint_line(int g) const {
    cout << "[g=" << g << ", ";
    statistics.print_basic_statistics();
    cout << "]" << endl;
}

void EagerSearch::print_statistics() const {
    statistics.print_detailed_statistics();
    search_space.print_statistics();
    pruning_method->print_statistics();
}

SearchStatus EagerSearch::step() {
    pair<SearchNode, bool> n = fetch_next_node();
    if (!n.second) {
        return FAILED;
    }
    SearchNode node = n.first;

    GlobalState s = node.get_state();
    if (check_goal_and_set_plan(s, group)) {
    	if (dump_forbid_plan_reformulation)
    		dump_plan_forbid_reformulation_sas("reformulated_output.sas", get_plan());
        	//dump_reformulated_sas("reformulated_output.sas");

        return FIRST_PLAN_FOUND;
    }

    vector<const GlobalOperator *> applicable_ops;
    g_successor_generator->generate_applicable_ops(s, applicable_ops);

    /*
      TODO: When preferred operators are in use, a preferred operator will be
      considered by the preferred operator queues even when it is pruned.
    */
    pruning_method->prune_operators(s, applicable_ops);

    // This evaluates the expanded state (again) to get preferred ops
    EvaluationContext eval_context(s, node.get_g(), false, &statistics, true);
    ordered_set::OrderedSet<const GlobalOperator *> preferred_operators =
        collect_preferred_operators(eval_context, preferred_operator_heuristics);

    for (const GlobalOperator *op : applicable_ops) {
        if ((node.get_real_g() + op->get_cost()) >= bound)
            continue;
        /*
          NOTE: In orbit search tmp_registry has to survive as long as
                succ_state is used. This could be forever, but for heuristics
                that do not store per state information it is ok to keep it
                only for this operator application. In regular search it is not
                actually needed, but I don't see a way around having it there,
                too.
        */
        StateRegistry tmp_registry(*g_root_task(), *g_state_packer,
                                   *g_axiom_evaluator, g_initial_state_data);
        StateRegistry *successor_registry = use_oss() ? &tmp_registry : &state_registry;
        GlobalState succ_state = successor_registry->get_successor_state(s, *op);
        if (use_oss()) {
            int *canonical_state = group->get_canonical_representative(succ_state);
            succ_state = state_registry.register_state_buffer(canonical_state);
            delete canonical_state;
        }
        statistics.inc_generated();
        bool is_preferred = preferred_operators.contains(op);

        SearchNode succ_node = search_space.get_node(succ_state);

        // Previously encountered dead end. Don't re-evaluate.
        if (succ_node.is_dead_end())
            continue;

        // update new path
        if (use_multi_path_dependence || succ_node.is_new()) {
            /*
              Note: we must call notify_state_transition for each heuristic, so
              don't break out of the for loop early.
            */
            for (Heuristic *heuristic : heuristics) {
                heuristic->notify_state_transition(s, *op, succ_state);
            }
        }

        if (succ_node.is_new()) {
            // We have not seen this state before.
            // Evaluate and create a new node.

            // Careful: succ_node.get_g() is not available here yet,
            // hence the stupid computation of succ_g.
            // TODO: Make this less fragile.
            int succ_g = node.get_g() + get_adjusted_cost(*op);

            /*
              NOTE: previous versions used the non-canocialized successor state
              here, but this lead to problems because the EvaluationContext was
              initialized with one state and the insertion was performed with
              another state.
             */
            EvaluationContext eval_context(
                succ_state, succ_g, is_preferred, &statistics);
            statistics.inc_evaluated_states();

            if (open_list->is_dead_end(eval_context)) {
                succ_node.mark_as_dead_end();
                statistics.inc_dead_ends();
                continue;
            }
            succ_node.open(node, op);

            open_list->insert(eval_context, succ_state.get_id());
            if (search_progress.check_progress(eval_context)) {
                print_checkpoint_line(succ_node.get_g());
                reward_progress();
            }
        } else if (succ_node.get_g() > node.get_g() + get_adjusted_cost(*op)) {
            // We found a new cheapest path to an open or closed state.
            if (reopen_closed_nodes) {
                if (succ_node.is_closed()) {
                    /*
                      TODO: It would be nice if we had a way to test
                      that reopening is expected behaviour, i.e., exit
                      with an error when this is something where
                      reopening should not occur (e.g. A* with a
                      consistent heuristic).
                    */
                    statistics.inc_reopened();
                }
                succ_node.reopen(node, op);

                EvaluationContext eval_context(
                    succ_state, succ_node.get_g(), is_preferred, &statistics);

                /*
                  Note: our old code used to retrieve the h value from
                  the search node here. Our new code recomputes it as
                  necessary, thus avoiding the incredible ugliness of
                  the old "set_evaluator_value" approach, which also
                  did not generalize properly to settings with more
                  than one heuristic.

                  Reopening should not happen all that frequently, so
                  the performance impact of this is hopefully not that
                  large. In the medium term, we want the heuristics to
                  remember heuristic values for states themselves if
                  desired by the user, so that such recomputations
                  will just involve a look-up by the Heuristic object
                  rather than a recomputation of the heuristic value
                  from scratch.
                */
                open_list->insert(eval_context, succ_state.get_id());
            } else {
                // If we do not reopen closed nodes, we just update the parent pointers.
                // Note that this could cause an incompatibility between
                // the g-value and the actual path that is traced back.
                succ_node.update_parent(node, op);
            }
        }
    }

    return IN_PROGRESS;
}

pair<SearchNode, bool> EagerSearch::fetch_next_node() {
    /* TODO: The bulk of this code deals with multi-path dependence,
       which is a bit unfortunate since that is a special case that
       makes the common case look more complicated than it would need
       to be. We could refactor this by implementing multi-path
       dependence as a separate search algorithm that wraps the "usual"
       search algorithm and adds the extra processing in the desired
       places. I think this would lead to much cleaner code. */

    while (true) {
        if (open_list->empty()) {
            cout << "Completely explored state space -- no solution!" << endl;
            // HACK! HACK! we do this because SearchNode has no default/copy constructor
            const GlobalState &initial_state = state_registry.get_initial_state();
            SearchNode dummy_node = search_space.get_node(initial_state);
            return make_pair(dummy_node, false);
        }
        vector<int> last_key_removed;
        StateID id = open_list->remove_min(
            use_multi_path_dependence ? &last_key_removed : nullptr);
        // TODO is there a way we can avoid creating the state here and then
        //      recreate it outside of this function with node.get_state()?
        //      One way would be to store GlobalState objects inside SearchNodes
        //      instead of StateIDs
        GlobalState s = state_registry.lookup_state(id);
        SearchNode node = search_space.get_node(s);

        if (node.is_closed())
            continue;

        if (use_multi_path_dependence) {
            assert(last_key_removed.size() == 2);
            if (node.is_dead_end())
                continue;
            int pushed_h = last_key_removed[1];

            if (!node.is_closed()) {
                EvaluationContext eval_context(
                    node.get_state(), node.get_g(), false, &statistics);

                if (open_list->is_dead_end(eval_context)) {
                    node.mark_as_dead_end();
                    statistics.inc_dead_ends();
                    continue;
                }
                if (pushed_h < eval_context.get_result(heuristics[0]).get_h_value()) {
                    assert(node.is_open());
                    open_list->insert(eval_context, node.get_state_id());
                    continue;
                }
            }
        }

        node.close();
        assert(!node.is_dead_end());
        update_f_value_statistics(node);
        statistics.inc_expanded();
        return make_pair(node, true);
    }
}

void EagerSearch::reward_progress() {
    // Boost the "preferred operator" open lists somewhat whenever
    // one of the heuristics finds a state with a new best h value.
    open_list->boost_preferred();
}

void EagerSearch::dump_search_space() const {
    search_space.dump();
}

void EagerSearch::start_f_value_statistics(EvaluationContext &eval_context) {
    if (f_evaluator) {
        int f_value = eval_context.get_heuristic_value(f_evaluator);
        statistics.report_f_value_progress(f_value);
    }
}

/* TODO: HACK! This is very inefficient for simply looking up an h value.
   Also, if h values are not saved it would recompute h for each and every state. */
void EagerSearch::update_f_value_statistics(const SearchNode &node) {
    if (f_evaluator) {
        /*
          TODO: This code doesn't fit the idea of supporting
          an arbitrary f evaluator.
        */
        EvaluationContext eval_context(node.get_state(), node.get_g(), false, &statistics);
        int f_value = eval_context.get_heuristic_value(f_evaluator);
        statistics.report_f_value_progress(f_value);
    }
}

/*
void EagerSearch::dump_reformulated_sas(const char* filename) const {
	int v_ind = g_variable_domain.size();
	ofstream os(filename);
	dump_version(os);
	dump_metric(os);

	// The variables are the original ones + n+2 binary variables for a plan of length n
	os << g_variable_domain.size() + get_plan().size() + 2 << endl;
	for(size_t i = 0; i < g_variable_domain.size(); ++i) {
		dump_variable(os, g_variable_name[i], g_variable_domain[i], g_fact_names[i]);
	}
	vector<string> vals;
	vals.push_back("false");
	vals.push_back("true");
	dump_variable(os, "possible", 2, vals);
	for(size_t i = 0; i <= get_plan().size(); ++i) {
		string name = "following" + static_cast<ostringstream*>( &(ostringstream() << i) )->str();
		dump_variable(os, name, 2, vals);
	}
	dump_mutexes(os);

	os << "begin_state" << endl;
	for(size_t i = 0; i < g_initial_state_data.size(); ++i)
		os << g_initial_state_data[i] << endl;
	os << 1 << endl;
	os << 1 << endl;
	for(size_t i = 0; i < get_plan().size(); ++i)
		os << 0 << endl;
	os << "end_state" << endl;

	os << "begin_goal" << endl;
	os << g_goal.size() + 1 << endl;
	for(size_t i = 0; i < g_goal.size(); ++i)
		os << g_goal[i].first << " " << g_goal[i].second << endl;
	os << v_ind << " " << 0 << endl;
	os << "end_goal" << endl;

	vector<bool> on_plan;
	int ops_on_plan = 0;
	on_plan.assign(g_operators.size(), false);
	for (const GlobalOperator* op : get_plan()) {
		int op_no = get_op_index_hacked(op);
		if (!on_plan[op_no]) {
			ops_on_plan++;
			on_plan[op_no] = true;
		}
	}

	// The operators are the original ones not on the plan + 3 operators for each on the plan
	os << g_operators.size()  - ops_on_plan + (3 * get_plan().size()) << endl;
	// The order of the operators might affect computation...
	// First dumping the original ones, that are not on the plan
	vector<GlobalCondition> empty_pre;
	vector<GlobalEffect> empty_eff;

	for(size_t op_no = 0; op_no < g_operators.size(); ++op_no) {
		if (on_plan[op_no])
			continue;

		vector<GlobalEffect> eff;
		eff.push_back(GlobalEffect(v_ind, 0, empty_pre, false));
		g_operators[op_no].dump_SAS(os, empty_pre, eff);
	}
	for(size_t op_no = 0; op_no < get_plan().size(); ++op_no) {
		const GlobalOperator* op = get_plan()[op_no];

		//Dumping operators on the plan
		vector<GlobalCondition> pre1, pre2, pre3;
		vector<GlobalEffect> eff2,eff3;

		pre1.push_back(GlobalCondition(v_ind, 0, false));
		op->dump_SAS(os, pre1, empty_eff);

		int following_var_from_ind = v_ind + 1 + op_no;
		pre2.push_back(GlobalCondition(v_ind, 1, false));
		pre2.push_back(GlobalCondition(following_var_from_ind, 0, false));
		eff2.push_back(GlobalEffect(v_ind, 0, empty_pre, false));
		op->dump_SAS(os, pre2, eff2);

		pre3.push_back(GlobalCondition(v_ind, 1, false));
		pre3.push_back(GlobalCondition(following_var_from_ind, 1, false));
		eff3.push_back(GlobalEffect(following_var_from_ind, 0, empty_pre, false));
		eff3.push_back(GlobalEffect(following_var_from_ind+1, 1, empty_pre, false));
		op->dump_SAS(os, pre3, eff3);
	}
	os << g_axioms.size() << endl;
	for(size_t op_no = 0; op_no < g_axioms.size(); ++op_no) {
		g_axioms[op_no].dump_SAS(os, empty_pre, empty_eff);
	}
}
*/
void add_forbid_plan_reformulation_option(OptionParser &parser) {
    parser.add_option<bool>("dump_forbid_plan_reformulation",
        "Dumping task reformulation that forbids the found plan",
        "false");
}



/* TODO: merge this into SearchEngine::add_options_to_parser when all search
         engines support pruning. */
void add_pruning_option(OptionParser &parser) {
    parser.add_option<shared_ptr<PruningMethod>>(
        "pruning",
        "Pruning methods can prune or reorder the set of applicable operators in "
        "each state and thereby influence the number and order of successor states "
        "that are considered.",
        "null()");
}

static SearchEngine *_parse(OptionParser &parser) {
    parser.document_synopsis("Eager best-first search", "");

    parser.add_option<shared_ptr<OpenListFactory>>("open", "open list");
    parser.add_option<bool>("reopen_closed",
                            "reopen closed nodes", "false");
    parser.add_option<ScalarEvaluator *>(
        "f_eval",
        "set evaluator for jump statistics. "
        "(Optional; if no evaluator is used, jump statistics will not be displayed.)",
        OptionParser::NONE);
    parser.add_list_option<Heuristic *>(
        "preferred",
        "use preferred operators of these heuristics", "[]");

    add_pruning_option(parser);
    add_forbid_plan_reformulation_option(parser);
    SearchEngine::add_options_to_parser(parser);
    Options opts = parser.parse();

    EagerSearch *engine = nullptr;
    if (!parser.dry_run()) {
        opts.set<bool>("mpd", false);
        engine = new EagerSearch(opts);
    }

    return engine;
}

static SearchEngine *_parse_astar(OptionParser &parser) {
    parser.document_synopsis(
        "A* search (eager)",
        "A* is a special case of eager best first search that uses g+h "
        "as f-function. "
        "We break ties using the evaluator. Closed nodes are re-opened.");
    parser.document_note(
        "mpd option",
        "This option is currently only present for the A* algorithm and not "
        "for the more general eager search, "
        "because the current implementation of multi-path depedence "
        "does not support general open lists.");
    parser.document_note(
        "Equivalent statements using general eager search",
        "\n```\n--search astar(evaluator)\n```\n"
        "is equivalent to\n"
        "```\n--heuristic h=evaluator\n"
        "--search eager(tiebreaking([sum([g(), h]), h], unsafe_pruning=false),\n"
        "               reopen_closed=true, f_eval=sum([g(), h]))\n"
        "```\n", true);
    parser.add_option<ScalarEvaluator *>("eval", "evaluator for h-value");
    parser.add_option<bool>("mpd",
                            "use multi-path dependence (LM-A*)", "false");

    add_pruning_option(parser);
    add_forbid_plan_reformulation_option(parser);
    SearchEngine::add_options_to_parser(parser);
    parser.add_option<shared_ptr<Group>>(
        "symmetries",
        "symmetries object to compute structural symmetries for pruning",
        OptionParser::NONE);

    Options opts = parser.parse();

    EagerSearch *engine = nullptr;
    if (!parser.dry_run()) {
        if (opts.contains("symmetries")) {
            shared_ptr<Group> group = opts.get<shared_ptr<Group>>("symmetries");
            if (group->get_search_symmetries() == SearchSymmetries::NONE) {
                cerr << "Symmetries option passed to eager search, but no "
                     << "search symmetries should be used." << endl;
                utils::exit_with(utils::ExitCode::INPUT_ERROR);
            }
        }

        auto temp = search_common::create_astar_open_list_factory_and_f_eval(opts);
        opts.set("open", temp.first);
        opts.set("f_eval", temp.second);
        opts.set("reopen_closed", true);
        vector<Heuristic *> preferred_list;
        opts.set("preferred", preferred_list);
        engine = new EagerSearch(opts);
    }

    return engine;
}

static SearchEngine *_parse_greedy(OptionParser &parser) {
    parser.document_synopsis("Greedy search (eager)", "");
    parser.document_note(
        "Open list",
        "In most cases, eager greedy best first search uses "
        "an alternation open list with one queue for each evaluator. "
        "If preferred operator heuristics are used, it adds an extra queue "
        "for each of these evaluators that includes only the nodes that "
        "are generated with a preferred operator. "
        "If only one evaluator and no preferred operator heuristic is used, "
        "the search does not use an alternation open list but a "
        "standard open list with only one queue.");
    parser.document_note(
        "Closed nodes",
        "Closed node are not re-opened");
    parser.document_note(
        "Equivalent statements using general eager search",
        "\n```\n--heuristic h2=eval2\n"
        "--search eager_greedy([eval1, h2], preferred=h2, boost=100)\n```\n"
        "is equivalent to\n"
        "```\n--heuristic h1=eval1 --heuristic h2=eval2\n"
        "--search eager(alt([single(h1), single(h1, pref_only=true), single(h2), \n"
        "                    single(h2, pref_only=true)], boost=100),\n"
        "               preferred=h2)\n```\n"
        "------------------------------------------------------------\n"
        "```\n--search eager_greedy([eval1, eval2])\n```\n"
        "is equivalent to\n"
        "```\n--search eager(alt([single(eval1), single(eval2)]))\n```\n"
        "------------------------------------------------------------\n"
        "```\n--heuristic h1=eval1\n"
        "--search eager_greedy(h1, preferred=h1)\n```\n"
        "is equivalent to\n"
        "```\n--heuristic h1=eval1\n"
        "--search eager(alt([single(h1), single(h1, pref_only=true)]),\n"
        "               preferred=h1)\n```\n"
        "------------------------------------------------------------\n"
        "```\n--search eager_greedy(eval1)\n```\n"
        "is equivalent to\n"
        "```\n--search eager(single(eval1))\n```\n", true);

    parser.add_list_option<ScalarEvaluator *>("evals", "scalar evaluators");
    parser.add_list_option<Heuristic *>(
        "preferred",
        "use preferred operators of these heuristics", "[]");
    parser.add_option<int>(
        "boost",
        "boost value for preferred operator open lists", "0");

    add_pruning_option(parser);
    add_forbid_plan_reformulation_option(parser);
    SearchEngine::add_options_to_parser(parser);

    Options opts = parser.parse();
    opts.verify_list_non_empty<ScalarEvaluator *>("evals");

    EagerSearch *engine = nullptr;
    if (!parser.dry_run()) {
        opts.set("open", search_common::create_greedy_open_list_factory(opts));
        opts.set("reopen_closed", false);
        opts.set("mpd", false);
        ScalarEvaluator *evaluator = nullptr;
        opts.set("f_eval", evaluator);
        engine = new EagerSearch(opts);
    }
    return engine;
}

static Plugin<SearchEngine> _plugin("eager", _parse);
static Plugin<SearchEngine> _plugin_astar("astar", _parse_astar);
static Plugin<SearchEngine> _plugin_greedy("eager_greedy", _parse_greedy);
}
