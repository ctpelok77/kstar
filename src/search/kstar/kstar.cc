#include "kstar.h"

#include "../plugin.h"
#include "../option_parser.h"
#include "../search_engines/search_common.h"
#include "../utils/util.h"
#include "../utils/countdown_timer.h"
#include "util.h"

using namespace top_k_eager_search;

namespace kstar{

KStar::KStar(const options::Options &opts) : TopKEagerSearch(opts),
        optimal_solution_cost(-1),
        simple_plans_only(opts.get<bool>("simple_plans_only")),
        dump_states(opts.get<bool>("dump_states")),
        dump_json(opts.contains("json_file_to_dump")),
        json_filename(""),
        num_node_expansions(0),
        djkstra_initialized(false) {
    if (dump_json) {
        json_filename = opts.get<string>("json_file_to_dump");
    }
    pg_succ_generator =
            unique_ptr<SuccessorGenerator>(new SuccessorGenerator(
                                                            tree_heap,
                                                            incomming_heap,
                                                            parent_node,
                                                            cross_edge,
                                                            &state_registry));
    plan_reconstructor =  unique_ptr<PlanReconstructor>(new PlanReconstructor(
                                                       parent_node,
                                                       cross_edge,
                                                       goal_state,
                                                       &state_registry,
                                                       &search_space, opts.get<bool>("skip_reorderings"), opts.get<bool>("dump_plans"), verbosity));

}

void KStar::search() {
    initialize();
    utils::CountdownTimer timer(max_time);
    while (status == IN_PROGRESS || status == INTERRUPTED
           || status == FIRST_PLAN_FOUND) {
        status = step();
        if (timer.is_expired()) {
            cout << "Time limit reached. Aborting search." << endl;
            status = TIMEOUT;
            plan_reconstructor->write_non_optimal_plans();
            break;
        }
        // First solution found. Add R to path graph, perform Dijkstra
        if (status == FIRST_PLAN_FOUND) {
            if (verbosity >= Verbosity::NORMAL) {
                cout << "[KSTAR] First plan is found" << endl;
            }            
            if (djkstra_search()) {
                if (verbosity >= Verbosity::NORMAL) {
                    cout << "[KSTAR] Dijkstra search finished successfully, found all required plans" << endl;
                }                
                status = SOLVED;
                solution_found = true;
                // Michael: need to break here
                // Here, non-optimal plans are written at the end of the successful execution of djkstra_search()
                break;
            }
            if (verbosity >= Verbosity::NORMAL) {
                cout << "[KSTAR] Dijkstra search could not find all required plans" << endl;
            }            
            interrupt();
        }

        // Check whether A* has expanded enough nodes and if yes
        // start a Dijkstra search on P(G)
        if (status == INTERRUPTED) {
            if (verbosity >= Verbosity::NORMAL) {
                cout << "[KSTAR] status INTERRUPTED" << endl;
            }
            // if (!open_list->empty() && !queue_djkstra.empty()) {
            //     // if enough nodes expanded do Dijkstra search
            //     if (verbosity >= Verbosity::NORMAL) {
            //         cout << "[KSTAR] open list not empty, dijkstra queue not empty" << endl;
            //     }
            //     if (enough_nodes_expanded()) {
            //         if (verbosity >= Verbosity::NORMAL) {
            //             cout << "[KSTAR] enough nodes are expanded" << endl;
            //         }
            //         if (djkstra_search()) {
            //             if (verbosity >= Verbosity::NORMAL) {
            //                 cout << "[KSTAR] Dijkstra search finished successfully, found all required plans" << endl;
            //             }
            //             status = SOLVED;
            //             solution_found = true;
            //             break;
            //         }
            //         if (verbosity >= Verbosity::NORMAL) {
            //             cout << "[KSTAR] Dijkstra search could not find all required plans" << endl;
            //         }
            //     } else {
            //         if (verbosity >= Verbosity::NORMAL) {
            //             cout << "[KSTAR] not enough nodes are expanded, resuming Astar" << endl;
            //         }
            //         resume_astar();
            //     }
            // }
            // if (!open_list->empty() && queue_djkstra.empty()) {
            //     if (verbosity >= Verbosity::NORMAL) {
            //         cout << "[KSTAR] open list not empty, dijkstra queue empty, resuming Astar" << endl;
            //     }
            //     resume_astar();
            // }

            // if (open_list->empty()) {
            //     if (verbosity >= Verbosity::NORMAL) {
            //         cout << "[KSTAR] Astar open list is empty, trying Dijkstra" << endl;
            //     }
            //     if (djkstra_search()) {
            //         if (verbosity >= Verbosity::NORMAL) {
            //             cout << "[KSTAR] Dijkstra search finished successfully, found all required plans" << endl;
            //         }
            //         status = SOLVED;
            //     } else {
            //         if (verbosity >= Verbosity::NORMAL) {
            //             cout << "[KSTAR] Dijkstra search could not find all required plans" << endl;
            //         }
            //         status = FAILED;
            //     }
            //     solution_found = true;
            //     break;
            // }

            // Michael: October 9, 2020. Rewriting the part above, running Dijkstra/A* after A* was interrupted
            // First, we try Dijkstra. If enough plans found, we are done. If not, if A* queue is not empty, we continue A*.
            if (djkstra_search()) {
                if (verbosity >= Verbosity::NORMAL) {
                    cout << "[KSTAR] Dijkstra search finished successfully, found all required plans" << endl;
                }
                status = SOLVED;
                solution_found = true;
                // Here, non-optimal plans are written at the end of the successful execution of djkstra_search()
                break;
            }
            if (verbosity >= Verbosity::NORMAL) {
                cout << "[KSTAR] Dijkstra search could not find all required plans" << endl;
            }
            if (!open_list->empty()) {
                if (verbosity >= Verbosity::NORMAL) {
                    cout << "[KSTAR] Astar open list not empty, resuming Astar" << endl;
                }
                resume_astar();
            } else {
                if (verbosity >= Verbosity::NORMAL) {
                    cout << "[KSTAR] Astar open list empty, no more solutions can be found" << endl;
                }
                status = FAILED;
                solution_found = true;
                plan_reconstructor->write_non_optimal_plans();
                break;
            }             
        }
    }

    /* Michael: UGLY HACK for the case of a single plan
     *
     * It seems like in the case there is only one plan for the problem, the Dijkstra step fails to reconstruct it.
     * As a result, after running the K* search, if there was a plan found, and there were no reconstructed plans, 
     * we add the first found plan manually.
     */

    if (plan_reconstructor->number_of_plans_found() == 0 && first_plan_found) {
        // There has to be at least one plan
        GlobalState g = state_registry.lookup_state(goal_state);

        Plan plan;
        search_space.trace_path(g, plan);
        plan_reconstructor->add_plan_explicit_no_check(plan);
        set_optimal_plan_cost(plan_reconstructor->get_last_added_plan_cost());
        inc_optimal_plans_count(plan_reconstructor->get_last_added_plan_cost());
        statistics.inc_plans_found();
    }

    if (dump_json) {
        ofstream os(json_filename.c_str());
        plan_reconstructor->dump_plans_json(os, dump_states);
    }

    cout << "Actual search time: " << timer
         << " [t=" << utils::g_timer << "]" << endl;
}

bool KStar::enough_nodes_expanded() {
    if (open_list->empty()) {
        if (verbosity >= Verbosity::VERBOSE) {
            cout << "[KSTAR] Open list is empty" << endl;
        }
        return true;
    }
    if (optimal_solution_cost == -1) {
        if (verbosity >= Verbosity::NORMAL) {
           cout << "[KSTAR] Optimal solution cost is -1" << endl;
        }
        return false;
    }
    update_most_expensive_succ();

    int max_plan_cost =  optimal_solution_cost + most_expensive_successor;
    if (verbosity >= Verbosity::NORMAL) {
        cout << "[KSTAR] Checking if enough nodes expanded. Max plan cost: " << max_plan_cost << ", optimal solution cost: " << optimal_solution_cost << ", next node f: " << next_node_f << endl;
    }    
    if (max_plan_cost <= next_node_f)
        return true;
    return false;
}

void KStar::resume_astar() {
    if (verbosity >= Verbosity::NORMAL) {
        cout << "[KSTAR] Resuming Astar" << endl;
    }
    interrupted = false;
}

void KStar::update_most_expensive_succ() {
    if(queue_djkstra.empty())
        return;
    Node n = queue_djkstra.top();
    most_expensive_successor = n.g + pg_succ_generator->get_max_successor_delta(n, pg_root);
}

void KStar::set_optimal_plan_cost(int plan_cost) {
    optimal_solution_cost = plan_cost;
}

void KStar::initialize_djkstra() {
    if(djkstra_initialized || goal_state == StateID::no_state)
        return;

    if (verbosity >= Verbosity::NORMAL) {
        cout << "[KSTAR] Initializing Dijkstra" << endl;
    }

    GlobalState g = state_registry.lookup_state(goal_state);
    plan_reconstructor->set_goal_state(goal_state);
    if (verbosity >= Verbosity::VERBOSE) {
        cout << "[KSTAR] Initializing tree heap for goal state" << endl;
        g.dump_pddl();
    }
    init_tree_heap(g);
    if (tree_heap[g].empty()) {
        return;
    }
    // Generate root of path graph
    if (verbosity >= Verbosity::NORMAL) {
        cout << "[KSTAR] Generating root of path graph" << endl;
    }

    Sap sap = make_shared<StateActionPair>(StateID::no_state,
                                           goal_state, nullptr,
                                           &state_registry, &search_space);
    pg_root = make_shared<Node>(0, sap, StateID::no_state);
    pg_root->id = g_djkstra_nodes;
    ++g_djkstra_nodes;
    if (verbosity >= Verbosity::NORMAL) {
        cout << "[KSTAR] Adding the first plan" << endl;
    }    
    bool added = plan_reconstructor->add_plan(*pg_root, simple_plans_only);
    // cout << "Plan was added: " << added << endl; 
    assert(added); // The first plan should always be successfully added
    set_optimal_plan_cost(plan_reconstructor->get_last_added_plan_cost());
    inc_optimal_plans_count(plan_reconstructor->get_last_added_plan_cost());
    statistics.inc_plans_found();
    Node successor;
    pg_succ_generator->get_successor_pg_root(pg_root, successor);
    queue_djkstra.push(successor);
    statistics.inc_total_djkstra_generations();
    djkstra_initialized = true;
}

bool KStar::enough_plans_found() const {
    return enough_plans_found_topk() || enough_plans_found_topq();
}

bool KStar::enough_plans_found_topk() const {
    return ( number_of_plans >0 && (int) plan_reconstructor->number_of_plans_found() >= number_of_plans);
}

bool KStar::enough_plans_found_topq() const {
    if (quality_bound < 1.0 || optimal_solution_cost == -1 || plan_reconstructor->number_of_plans_found() == 0)
        return false;

    int cost = plan_reconstructor->get_last_added_plan_cost();
    double actual_bound = optimal_solution_cost * quality_bound;
    return (cost > actual_bound);
}


// init the neccessary tree heaps for the successor generation
// of s that is tree_heap[s] (obviously) and from (for the cross edge)
void KStar::init_tree_heaps(Node node) {
    GlobalState s = state_registry.lookup_state(node.heap_state);
    init_tree_heap(s);
    GlobalState from  = state_registry.lookup_state(node.sap->from);
    init_tree_heap(from);
}

void KStar::throw_everything() {
    djkstra_initialized = false;
    if (verbosity >= Verbosity::NORMAL) {
        cout << "[KSTAR] Before throwing everything we had " << plan_reconstructor->number_of_plans_found() << " plans" << endl;
    }
    plan_reconstructor->clear();

    num_node_expansions = 0;
    statistics.reset_plans_found();
    statistics.reset_opt_found();
    queue_djkstra = std::priority_queue<Node>();
}

// Djkstra search on path graph P(G) returns true if enough plans have been found
bool KStar::djkstra_search() {
    if (verbosity >= Verbosity::NORMAL) {
        std::cout << "[KSTAR] Switching to djkstra search on path graph" << std::endl;
    }
    // When Djkstra restarts remove everything from its last iteration
    throw_everything();
    statistics.inc_djkstra_runs();
    initialize_djkstra();
    if (verbosity >= Verbosity::NORMAL) {
        dump_dot();
    }
    int succ_gens = 0;
    int exps = 0;
    if (verbosity >= Verbosity::NORMAL) {
        cout << "[KSTAR] Start reconstructing plans using Dijkstra" << endl;
    }
    while (!queue_djkstra.empty()) {
        Node node = queue_djkstra.top();
        if (!enough_nodes_expanded()) {
            if (verbosity >= Verbosity::NORMAL) {
                cout << "[KSTAR] Not enough nodes are expanded by Astar" << endl;
            }
            return false;
        }
        queue_djkstra.pop();

        //notify_expand(node, &state_registry, num_node_expansions);
        if (verbosity >= Verbosity::NORMAL) {
            cout << "[KSTAR] Getting a plan for the node " << node.id;
        }
        if (plan_reconstructor->add_plan(node, simple_plans_only)) {
            if (verbosity >= Verbosity::NORMAL) {
                cout << "  added with cost" << plan_reconstructor->get_last_added_plan_cost() << endl;
            }
            inc_optimal_plans_count(plan_reconstructor->get_last_added_plan_cost());
            statistics.inc_plans_found();
        } else {
            if (verbosity >= Verbosity::NORMAL) {
                cout << "  Duplicate, not added" << endl;
            }
        }
        if (enough_plans_found()) {
            if (verbosity >= Verbosity::NORMAL) {
                cout << "Number of plans found: " << plan_reconstructor->number_of_plans_found() << endl;
            }
            // Writing the non-optimal plans found
            plan_reconstructor->write_non_optimal_plans();
            return true;
        }
        exps++;
        init_tree_heaps(node);
        std::vector<Node> successors;
        pg_succ_generator->get_successors(node, successors);
        succ_gens += successors.size();
        for (auto succ : successors) {
            queue_djkstra.push(succ);
            statistics.inc_total_djkstra_generations();
        }
        if (verbosity >= Verbosity::NORMAL) {
            if (succ_gens % 1000 == 0) {
                std::cout << "[KSTAR] Djkstra ["<< exps << " expanded, "<< succ_gens << " generated]" << std::endl;
            }
        }
    }
    if (verbosity >= Verbosity::NORMAL) {
        cout << "Number of plans found: " << plan_reconstructor->number_of_plans_found() << endl;
    }
    return false;
}

void KStar::inc_optimal_plans_count(int plan_cost) {
    if (plan_cost == optimal_solution_cost && plan_cost >= 0) {
        statistics.inc_opt_plans();
    }
}

void KStar::dump_path_graph() {
    std::stringstream stream, node_stream;
    std::string filename = "path_graph.dot";
    stream << "digraph {\n" << endl ;
    PerStateInformation<SearchNodeInfo>& search_node_infos =
            search_space.search_node_infos;
    for (PerStateInformation<SearchNodeInfo>::const_iterator it =\
         search_node_infos.begin(&state_registry);
         it != search_node_infos.end(&state_registry); ++it) {
        StateID id = *it;
        GlobalState s = state_registry.lookup_state(id);
        begin_subgraph(s.get_state_tuple(), stream);
        for (size_t i = 0; i  < tree_heap[s].size(); ++i){
            Sap sap = tree_heap[s][i];
            std::string id = get_sap_id(sap, s);
            std::string label = get_sap_label(sap);
            Node node(0, sap, s.get_id());
            vector<Node> successors;
            pg_succ_generator->get_successors(node, successors, true);
            for (auto& succ : successors) {
                GlobalState heap_state =
                        state_registry.lookup_state(succ.heap_state);
                std::string succ_id = get_sap_id(succ.sap, heap_state);
                add_edge(id, succ_id, std::to_string(succ.g), stream);
            }
        }
        stream << "}" << endl ;
    }
    save_and_close(filename, stream, node_stream);
}

void KStar::dump_dot() const {
    std::stringstream stream, node_stream;
    stream << "digraph {\n";
    for (PerStateInformation<SearchNodeInfo>::const_iterator it =\
            search_space.search_node_infos.begin(&state_registry);
             it != search_space.search_node_infos.end(&state_registry); ++it) {
        StateID id = *it;
        stream << id.hash() << " [ peripheries=\"1\", shape=\"rectangle\", ";
        GlobalState s = state_registry.lookup_state(id);
        const SearchNodeInfo &node_info = search_space.search_node_infos[s];

        if (test_goal(s)) {
            stream << "style=\"rounded, filled\", fillcolor=\"red\", ";
        }
        else {
            stream << "style=\"rounded, filled\", fillcolor=\"yellow\", ";
        }
        stream << "label=\"#"<< id.hash() << "\\n" << "s="<< state_label(s) << "\\n";
        stream << "\" ]\n";

        for (Sap sap: incomming_heap[s]) {
            node_stream << id.hash() << "  ->  " << sap->get_from_state().get_id().hash();
            node_stream <<" [ label=\"" << sap->op->get_name();
            node_stream << "/"<< sap->op->get_cost();
            node_stream << "\"";
            if (node_info.parent_state_id == sap->get_from_state().get_id()) {
                node_stream << " style=\"dashed\" color=\"#A9A9A9 \"";
            }
            node_stream << " ]\n";
        }
    }

    node_stream << "}\n";

    // Write state space to dot file
    std::ofstream file;
    file.open("full_state_space" + std::to_string(statistics.get_num_djkstra_runs()) + ".dot", std::ofstream::out);
    file << stream.rdbuf();
    file << node_stream.rdbuf();
    file.close();
}

void KStar::dump_tree_edge() {
    std::stringstream stream;
    std::string filename = "tree_edges";
    for (PerStateInformation<SearchNodeInfo>::const_iterator it =\
         search_space.search_node_infos.begin(&state_registry);
         it != search_space.search_node_infos.end(&state_registry); ++it) {
        StateID id = *it;
        GlobalState s = state_registry.lookup_state(id);
        const SearchNodeInfo &node_info = search_space.search_node_infos[s];
        if (node_info.creating_operator != -1
            && node_info.parent_state_id != StateID::no_state) {
            stream << s.get_state_tuple();
            stream << ": ";
            stream << g_operators[node_info.creating_operator].get_name();
        }
        stream << ""<< endl;
    }

    std::ofstream file;
    file.open(filename, std::ofstream::out);
    file << stream.rdbuf();
    file.close();
}


void add_simple_plans_only_option(OptionParser &parser) {
    parser.add_option<bool>("simple_plans_only", "", "false");
}


static SearchEngine *_parse(OptionParser &parser) {
    parser.add_option<ScalarEvaluator *>("eval", "evaluator for h-value");

    top_k_eager_search::add_top_k_option(parser);

    parser.add_option<bool>("dump_plans", "Print plans", "false");
    parser.add_option<bool>("dump_states", "Dump states to json", "false");
    
    parser.add_option<string>("json_file_to_dump",
        "A path to the json file to use for dumping",
        OptionParser::NONE);

    vector<string> verbosity_levels;
    vector<string> verbosity_level_docs;
    verbosity_levels.push_back("silent");
    verbosity_level_docs.push_back(
        "silent: no output during construction, only starting and final "
        "statistics");
    verbosity_levels.push_back("normal");
    verbosity_level_docs.push_back(
        "normal: basic output during construction, starting and final "
        "statistics");
    verbosity_levels.push_back("verbose");
    verbosity_level_docs.push_back(
        "verbose: full output during construction, starting and final "
        "statistics");
    parser.add_enum_option(
        "verbosity",
        verbosity_levels,
        "Option to specify the level of verbosity.",
        "silent",
        verbosity_level_docs);

    top_k_eager_search::add_pruning_option(parser);
    add_simple_plans_only_option(parser);
    SearchEngine::add_options_to_parser(parser);
    Options opts = parser.parse();
    KStar *engine = nullptr;
    if (!parser.dry_run()) {
        int num_plans = opts.get<int>("k");
        cout << "Running K* with K=" << num_plans << endl;
        auto temp = search_common::create_astar_open_list_factory_and_f_eval(opts);
        opts.set("open", temp.first);
        opts.set("f_eval", temp.second);
        opts.set("reopen_closed", true);
        std::vector<Heuristic *> preferred_list;
        opts.set("preferred", preferred_list);
        //opts.set("K", num_plans);

        engine = new KStar(opts);
    }

    return engine;
}

static Plugin<SearchEngine> _plugin("kstar", _parse);
}
