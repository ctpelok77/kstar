#include "top_k_eager_search.h"

#include "search_common.h"

#include "../evaluation_context.h"
#include "../globals.h"
#include "../heuristic.h"
#include "../option_parser.h"
#include "../plugin.h"
#include "../pruning_method.h"
#include "../successor_generator.h"
#include "../utils/util.h"
#include "../algorithms/ordered_set.h"
#include "../open_lists/open_list_factory.h"

#include <cassert>
#include <cstdlib>
#include <memory>
#include <string>
#include <set>

using namespace std;

namespace top_k_eager_search {
TopKEagerSearch::TopKEagerSearch(const Options &opts)
    : SearchEngine(opts),
      reopen_closed_nodes(opts.get<bool>("reopen_closed")),
      number_of_plans(opts.get<int>("K")),
      open_list(opts.get<shared_ptr<OpenListFactory>>("open")->
                create_state_open_list()),
      f_evaluator(opts.get<ScalarEvaluator *>("f_eval", nullptr)),
      preferred_operator_heuristics(opts.get_list<Heuristic *>("preferred")),
      pruning_method(opts.get<shared_ptr<PruningMethod>>("pruning")),
	  interrupt_search(false) {
}

void TopKEagerSearch::initialize() {
    cout << "Conducting best first search"
         << (reopen_closed_nodes ? " with" : " without")
         << " reopening closed nodes, (real) bound = " << bound
         << endl;
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

    const GlobalState &initial_state = state_registry.get_initial_state();
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

void TopKEagerSearch::print_checkpoint_line(int g) const {
    cout << "[g=" << g << ", ";
    statistics.print_basic_statistics();
    cout << "]" << endl;
}

void TopKEagerSearch::print_statistics() const {
    statistics.print_detailed_statistics();
    search_space.print_statistics();
    pruning_method->print_statistics();
}

SearchStatus TopKEagerSearch::step() {
	if (interrupt_search) 
		return INTERRUPTED;

    pair<SearchNode, bool> n = fetch_next_node();
    if (!n.second) {
        return FAILED;
    }
    SearchNode node = n.first;

    GlobalState s = node.get_state();
    if (test_goal(s)) {
		goal_state = s.get_id();
        return SOLVED;
	}	
	std::cout << "Expanding node s_"<< s[0] << std::endl;

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

        GlobalState succ_state = state_registry.get_successor_state(s, *op);
        statistics.inc_generated();
        bool is_preferred = preferred_operators.contains(op);

        SearchNode succ_node = search_space.get_node(succ_state);
		std::cout << "Inserting edge " << "(" << node.get_state()[0] << "," 
				  << succ_node.get_state()[0]  << ")"<< std::endl;

		update_path_graph(node, op ,succ_node);	

        // Previously encountered dead end. Don't re-evaluate.
        if (succ_node.is_dead_end())
            continue;

        // update new path
        if (succ_node.is_new()) {
			std::cout << "Generating node s_"<< succ_state[0] << std::endl;
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


void TopKEagerSearch::update_path_graph(SearchNode& node, 
										const GlobalOperator* op,
										SearchNode& succ_node) {
	// For K* we store the state action pair that leads to succ node
	// in succ nodes heap H_in 
	StateActionPair p(node.get_state_id(), succ_node.get_state_id(),op, 
					  &state_registry, &search_space);
	GlobalState succ_state = succ_node.get_state();
	H_in[succ_state].push(p);
}

// Dump incomming heaps in graphviz format
void TopKEagerSearch::dump_incomming_heaps() {
	std::stringstream stream, node_stream;	
	int total_num_nodes = 0;
	stream << "digraph {\n" << endl ;			
	PerStateInformation<SearchNodeInfo>& search_node_infos = search_space.search_node_infos;
	for (PerStateInformation<SearchNodeInfo>::const_iterator it =\
		search_node_infos.begin(&state_registry); 
		it != search_node_infos.end(&state_registry); ++it) {
		StateID id = *it;
		GlobalState s = state_registry.lookup_state(id); 
		stream << "subgraph " << "cluster_" <<  s[0] << " {" << endl; 	
		stream << "label=" << "node" <<  s[0] << ";" <<  endl;
		stream << "style=filled;" << endl;
		stream << "color=grey;"  << endl;
		// HACK:
		add_node("9000" + std::to_string(total_num_nodes), "none", stream);

		std::string node_name, node_label; 
		bool node_has_pred = false;
		StateActionPair pred = StateActionPair::no_sap;
		while (!H_in[s].empty()) {
			StateActionPair top_edge = H_in[s].top();
			node_label = get_node_label(top_edge);
			node_name = get_node_name(top_edge);
			bool node_added = false; 
			if (H_in[s].top().get_delta() != 0 ) {
				add_node(node_name, node_label, stream);
				node_added = true;
			}
			H_in[s].pop();
			if (node_has_pred) {
				std::string pred_name = get_node_name(pred);
				add_edge(pred_name, node_name, "" , stream);
			}

			if (node_added) 
				node_has_pred = true;

			pred = top_edge; 
			++total_num_nodes;
		} 
		
		stream << "}" << endl; 
	}
	stream << "}" << endl; 	
	std::ofstream file;
	file.open("inc_graph.dot", std::ofstream::out);
	file << stream.rdbuf();
	file << node_stream.rdbuf();
	file.close();
}

// Dump path_graph in graphviz format
void TopKEagerSearch::dump_path_graph() {
	std::stringstream stream, node_stream;	
	int total_num_nodes = 0;
	stream << "digraph {\n" << endl ;			
	PerStateInformation<SearchNodeInfo>& search_node_infos = search_space.search_node_infos;
	for (PerStateInformation<SearchNodeInfo>::const_iterator it =\
		search_node_infos.begin(&state_registry); 
		it != search_node_infos.end(&state_registry); ++it) {
		StateID id = *it;
		GlobalState s = state_registry.lookup_state(id); 
		// TODO: set parent
		if (!H_in[s].empty()) {
			H_T[s].set_root(H_in[s].top());
			
		}
		else {
			
		}
		StateID parent_id = search_space.search_node_infos[s].parent_state_id;
		GlobalState parent_state = state_registry.lookup_state(parent_id);
		H_T[s].set_ancestor_heap(&H_in[parent_state]);
		stream << "subgraph " << "cluster_" <<  s[0] << " {" << endl; 	
		stream << "label=" << "node" <<  s[0] << ";" <<  endl;
		stream << "style=filled;" << endl;
		stream << "color=grey;"  << endl;
		// HACK:
		add_node("9000" + std::to_string(total_num_nodes), "none", stream);

		std::string node_name, node_label; 
		bool node_has_pred = false;
		StateActionPair pred = StateActionPair::no_sap;
		debug(1000, _ARGS);
		print_value(H_T[s].empty(),"empty()", _ARGS);
		std::cout << "s=" << s[0] << std::flush << std::endl;
		debug(1000, _ARGS);
		while (!H_T[s].empty()) {
			debug(1, _ARGS);
			StateActionPair top_edge = H_in[s].top();
			debug(2, _ARGS);
			node_label = get_node_label(top_edge);
			node_name = get_node_name(top_edge);
			bool node_added = false; 
			if (H_T[s].top().get_delta() != 0 ) {
				add_node(node_name, node_label, stream);
				node_added = true;
			}
			H_T[s].pop();
			debug(4, _ARGS);
			if (node_has_pred) {
				std::string pred_name = get_node_name(pred);
				add_edge(pred_name, node_name, "" , stream);
			}

			if (node_added) 
				node_has_pred = true;

			pred = top_edge; 
			++total_num_nodes;
		} 
		
		stream << "}" << endl; 
	}
	stream << "}" << endl; 	
	std::ofstream file;
	file.open("path_graph.dot", std::ofstream::out);
	file << stream.rdbuf();
	file << node_stream.rdbuf();
	file.close();
}

std::string TopKEagerSearch::get_node_label(StateActionPair &edge) {
	int from = state_registry.lookup_state(edge.from)[0];
	int to = state_registry.lookup_state(edge.to)[0];
	std::string node_name = std::to_string(from) + std::to_string(to)
							+ " delta: " + std::to_string(edge.get_delta()); 
	return node_name;
}

std::string TopKEagerSearch::get_node_name(StateActionPair &edge) {
	int from = state_registry.lookup_state(edge.from)[0];
	int to = state_registry.lookup_state(edge.to)[0];
	std::string node_name = std::to_string(from) + std::to_string(to);
	return node_name;
}

int TopKEagerSearch::get_cost_heap_edge(StateActionPair& from, StateActionPair& to) {
	int cost_heap_edge = to.get_delta() -from.get_delta();
	assert(cost_heap_edge >= 0);
	return cost_heap_edge;	
}

int TopKEagerSearch::get_cost_cross_edge(StateActionPair&, StateActionPair& to) {
	int cost_cross_edge = to.get_delta();		
	assert(cost_cross_edge >= 0);
	return cost_cross_edge;
}

void TopKEagerSearch::interrupt() {
	status = INTERRUPTED;
	interrupt_search =  true;			
}

void TopKEagerSearch::resume(SearchControl& search_control) {
	(void) search_control;
	status = IN_PROGRESS;
	interrupt_search = false;	
}

pair<SearchNode, bool> TopKEagerSearch::fetch_next_node() {
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
        StateID id = open_list->remove_min(nullptr);
        // TODO is there a way we can avoid creating the state here and then
        //      recreate it outside of this function with node.get_state()?
        //      One way would be to store GlobalState objects inside SearchNodes
        //      instead of StateIDs
        GlobalState s = state_registry.lookup_state(id);
        SearchNode node = search_space.get_node(s);

        if (node.is_closed())
            continue;

        node.close();
        assert(!node.is_dead_end());
        update_f_value_statistics(node);
        statistics.inc_expanded();
        return make_pair(node, true);
    }
}

void TopKEagerSearch::reward_progress() {
    // Boost the "preferred operator" open lists somewhat whenever
    // one of the heuristics finds a state with a new best h value.
    open_list->boost_preferred();
}

void TopKEagerSearch::start_f_value_statistics(EvaluationContext &eval_context) {
    if (f_evaluator) {
        int f_value = eval_context.get_heuristic_value(f_evaluator);
        statistics.report_f_value_progress(f_value);
    }
}

/* TODO: HACK! This is very inefficient for simply looking up an h value.
   Also, if h values are not saved it would recompute h for each and every state. */
void TopKEagerSearch::update_f_value_statistics(const SearchNode &node) {
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

void add_top_k_option(OptionParser &parser) {
    parser.add_option<int>("K", "Number of plans", "10");
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

    add_top_k_option(parser);
    add_pruning_option(parser);
    SearchEngine::add_options_to_parser(parser);
    Options opts = parser.parse();

    TopKEagerSearch *engine = nullptr;
    if (!parser.dry_run()) {
        engine = new TopKEagerSearch(opts);
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
        "Equivalent statements using general eager search",
        "\n```\n--search astar(evaluator)\n```\n"
        "is equivalent to\n"
        "```\n--heuristic h=evaluator\n"
        "--search eager(tiebreaking([sum([g(), h]), h], unsafe_pruning=false),\n"
        "               reopen_closed=true, f_eval=sum([g(), h]))\n"
        "```\n", true);
    parser.add_option<ScalarEvaluator *>("eval", "evaluator for h-value");

    add_top_k_option(parser);
    add_pruning_option(parser);
    SearchEngine::add_options_to_parser(parser);
    Options opts = parser.parse();

    TopKEagerSearch *engine = nullptr;
    if (!parser.dry_run()) {
        auto temp = search_common::create_astar_open_list_factory_and_f_eval(opts);
        opts.set("open", temp.first);
        opts.set("f_eval", temp.second);
        opts.set("reopen_closed", true);
        vector<Heuristic *> preferred_list;
        opts.set("preferred", preferred_list);
        engine = new TopKEagerSearch(opts);
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

    add_top_k_option(parser);
    add_pruning_option(parser);
    SearchEngine::add_options_to_parser(parser);

    Options opts = parser.parse();
    opts.verify_list_non_empty<ScalarEvaluator *>("evals");

    TopKEagerSearch *engine = nullptr;
    if (!parser.dry_run()) {
        opts.set("open", search_common::create_greedy_open_list_factory(opts));
        opts.set("reopen_closed", false);
        ScalarEvaluator *evaluator = nullptr;
        opts.set("f_eval", evaluator);
        engine = new TopKEagerSearch(opts);
    }
    return engine;
}
void TopKEagerSearch::output_plans() {
	print_value(top_k_plans.size(),"size", _ARGS);
	for (size_t i = 0; i < top_k_plans.size(); ++i) {
		print_in_red("Begin Plan "+ std::to_string(i));
		print_plan(top_k_plans[i], false);									
		print_in_red("End Plan "+ std::to_string(i));
	}	
}

void TopKEagerSearch::print_plan(Plan plan, bool generates_multiple_plan_files) {
		
	ostringstream filename;
    filename << g_plan_filename;
    int plan_number = g_num_previously_generated_plans + 1;
    if (generates_multiple_plan_files || g_is_part_of_anytime_portfolio) {
        filename << "." << plan_number;
    } else {
        assert(plan_number == 1);
    }

    ofstream outfile(filename.str());
    for (size_t i = 0; i < plan.size(); ++i) {
        cout << plan[i]->get_name() << " (" << plan[i]->get_cost() << ")" << endl;
        outfile << "(" << plan[i]->get_name() << ")" << endl;
    }
    int plan_cost = calculate_plan_cost(plan);
    outfile << "; cost = " << plan_cost << " ("
            << (is_unit_cost() ? "unit cost" : "general cost") << ")" << endl;
    outfile.close();
    cout << "Plan length: " << plan.size() << " step(s)." << endl;
    cout << "Plan cost: " << plan_cost << endl;
    ++g_num_previously_generated_plans;
}

static Plugin<SearchEngine> _plugin("top_k_eager", _parse);
static Plugin<SearchEngine> _plugin_astar("top_k_astar", _parse_astar);
static Plugin<SearchEngine> _plugin_greedy("top_k_eager_greedy", _parse_greedy);
}
