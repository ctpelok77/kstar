#include "a_star_search.h"
#include "eager_greedy_best_first_search.h"
#include "cyclic_cg_heuristic.h"
#include "cg_heuristic.h"
#include "ff_heuristic.h"
#include "fd_heuristic.h"
#include "lm_cut_heuristic.h"
#include "max_heuristic.h"
#include "additive_heuristic.h"
#include "goal_count_heuristic.h"
#include "blind_search_heuristic.h"
#include "globals.h"
#include "structural_patterns/SP_globals.h"
//#include "structural_patterns/schema_globals.h"
#include "structural_patterns/LP_heuristic.h"
#include "structural_patterns/offline_heuristic.h"
#include "structural_patterns/online_heuristic.h"
#include "structural_patterns/init_opt_heuristic.h"
#include "problem.h"
#include "operator.h"
#include "timer.h"
#include "general_eager_best_first_search.h"
#include "landmarks/lama_ff_synergy.h"
#include "landmarks/landmarks_graph.h"
#include "landmarks/landmarks_graph_rpg_sasp.h"
#include "landmarks/landmarks_count_heuristic.h"
#include "landmarks/exploration.h"
#include "hm_heuristic.h"
#include "general_lazy_best_first_search.h"
#include "lazy_best_first_search_engine.h"
#include "lazy_wa_star.h"
#include "learning/selective_max_heuristic.h"
#include "learning/maximum_heuristic.h"
#include "learning/state_space_sample.h"
#include "learning/probe_state_space_sample.h"
#include "enforced_hill_climbing_search.h"
#include "structural_patterns/state_opt_heuristic.h"
#include "learning/selective_partition_heuristic.h"
#include <iostream>
#include <fstream>
#include <limits>
#include <vector>

using namespace std;

//int save_plan(const vector<const Operator *> &plan);
int save_plan(const vector<const Operator *> &plan, const string& filename);

int main(int argc, const char **argv) {
    srand(2010);
    bool poly_time_method = false;

    bool a_star_search = false;
    bool cg_heuristic = false, cg_preferred_operators = false;
    bool cyclic_cg_heuristic = false, cyclic_cg_preferred_operators = false;
    bool ff_heuristic = false, ff_preferred_operators = false;
    bool additive_heuristic = false, additive_preferred_operators = false;
    bool fd_heuristic = false;
    bool hsp_max_heuristic = false;
    bool goal_count_heuristic = false;
    bool blind_search_heuristic = false;
    bool lm_cut_heuristic = false;
    bool use_gen_search = false;
    bool use_lazy_search = false;
    bool use_wa_star = false;
    bool iterative_search = false;
    int weight = 0;
    bool lm_heuristic = false;
    bool lm_heuristic_admissible = false;
    bool lm_heuristic_optimal = false;
    bool lm_preferred = false;
    bool use_hm = false;
    int m_hm = 2;
    int lm_type = LandmarksCountHeuristic::rpg_sasp;
    bool use_selective_max = false;
    bool use_ehc_search = false;
    bool ehc_rank_by_preferred = false;
    bool test_mode = false;
    bool use_initial_optimal = false;

    bool SP_heuristic = false;
    bool lm_enriched_heuristic = false;
    int lm_enriched_heuristic_type = GENERAL_VARIABLES;
    bool maximum_heuristic = false;
    state_space_sample_t sample_mode = PDB;
    int num_clusters = 1;
    bool use_state_var_features = false;
    bool use_uniform_sp_features = false;

#ifdef USE_LP
    SelectivePartitionHeuristic::partition_type_t partition = SelectivePartitionHeuristic::cluster;
#endif

    string plan_filename = "sas_plan";
    if(argc < 3 || argc > 4) {
	std::cout << "Usage: \"search search_options heuristic_options [outputfile]\"\n";
    }
    else {
    	if (argc == 4) {
    		plan_filename = argv[3];
    	}
//    for (int i = 1; i < argc-1; i++) {
        for (const char *c = argv[1]; *c != 0; c++) {
            if (*c == 'o') {
                a_star_search = true; // "o"ptimal
            } else if (*c == 'e') {
                use_ehc_search = true; // "e"enforced hill climbing
            } else if (*c == 'r') {
                ehc_rank_by_preferred = true;
            } else if (*c == 'c') {
                cg_heuristic = true;
            } else if (*c == 'C') {
                cg_preferred_operators = true;
            } else if (*c == 'y') {
                cyclic_cg_heuristic = true;
            } else if (*c == 'Y') {
                cyclic_cg_preferred_operators = true;
            } else if (*c == 'f') {
                ff_heuristic = true;
            } else if (*c == 'F') {
                ff_preferred_operators = true;
            } else if (*c == 'a') {
                fd_heuristic = true;
            } else if (*c == 'k') {
                use_gen_search = true;
            } else if (*c == 'z') {
                use_lazy_search = true;
            } else if (*c == 'i') {
                iterative_search = true;
            } else if (*c == 'I') {
                use_initial_optimal = true;
            } else if (*c == 'w') {
                use_wa_star = true;
                c++;
                weight = ::atoi(c);
                while (*c >= '0' && *c <= '9')
                    c++;
                c--;
            } else if (*c == 'm') {
                hsp_max_heuristic = true;
            } else if (*c == 'h') {
                use_hm = true;
                c++;
                m_hm = ::atoi(c);
                while (*c >= '0' && *c <= '9')
                    c++;
                c--;
            } else if (*c == 'd') {
                additive_heuristic = true;
            } else if (*c == 'l') {
                lm_heuristic = true;
                c++;
                lm_type = ::atoi(c);
                while (*c >= '0' && *c <= '9')
                    c++;
                c--;
            } else if (*c == 's') {
                lm_heuristic_admissible = true;
            } else if (*c == 'p') {
                lm_heuristic_optimal = true;
            } else if (*c == 'P') {
            	g_do_path_dependent_search = true;
            } else if (*c == 'L') {
                lm_preferred = true;
            } else if (*c == 'M') {
                use_selective_max = true;
            } else if (*c == 'D') {
                additive_preferred_operators = true;
            } else if (*c == 'g') {
                goal_count_heuristic = true;
            } else if (*c == 'b') {
                blind_search_heuristic = true;
            } else if (*c == 'u') {
                lm_cut_heuristic = true;
            } else if (*c == 'X') {
                SP_heuristic = true;
            } else if (*c == 'U') {
                c++;
                sample_mode = (state_space_sample_t) ::atoi(c);
                cout << "Sample Mode: " << sample_mode << endl;
                while (*c >= '0' && *c <= '9')
                    c++;
                c--;
            } else if (*c == 'K') {
            	maximum_heuristic = true;
            	c++;
            	num_clusters = ::atoi(c);
				while (*c >= '0' && *c <= '9')
					c++;
				c--;
                cout << "Number of clusters: " << num_clusters << endl;
            } else if (*c == 'H') {
                c++;
#ifdef USE_LP
                partition = (SelectivePartitionHeuristic::partition_type_t) ::atoi(c);
                cout << "Partition Type: " << partition << endl;
#endif
                while (*c >= '0' && *c <= '9')
                    c++;
                c--;
            } else if (*c == 't') {
                use_state_var_features = true;
            } else if (*c == 'q') {
                use_uniform_sp_features = true;
            } else if (*c == 'T') {
				test_mode = true;
            } else if (*c == 'G') {
                lm_enriched_heuristic = true;
                lm_enriched_heuristic_type = SINGLE_GOAL;
            } else if (*c == 'E') {
                lm_enriched_heuristic = true;
                lm_enriched_heuristic_type = GENERAL_VARIABLES;
            } else if (*c == 'N') {
                lm_enriched_heuristic = true;
                lm_enriched_heuristic_type = ADDITIVE_ACTION_LANDMARKS;
            } else if (*c == 'Q') {
                lm_enriched_heuristic = true;
                lm_enriched_heuristic_type = VARIABLE_PER_LANDMARK_NO_INIT;
            } else if (*c == 'V') {
                lm_enriched_heuristic = true;
                lm_enriched_heuristic_type = VARIABLE_PER_LANDMARK;
            } else if (*c == 'W') {
                lm_enriched_heuristic = true;
                lm_enriched_heuristic_type = LANDMARK_PATHS_COVERING_VARIABLES;
            } else if (*c == 'Z') {
                lm_enriched_heuristic = true;
//                lm_enriched_heuristic_type = LANDMARKPATHSVARIABLES;
                lm_enriched_heuristic_type = ALL_LANDMARK_PATHS_VARIABLES;
            } else if (*c >= '0' && *c <= '9') {
                g_abstraction_max_size = ::atoi(c);
                while (*c >= '0' && *c <= '9')
                    c++;
                c--;
                if (g_abstraction_max_size < 1) {
                    cerr << "error: abstraction size must be at least 1"
                         << endl;
                    return 2;
                }
            } else if (*c == 'A') {
                c++;
                g_abstraction_nr = ::atoi(c);
                while (*c >= '0' && *c <= '9')
                    c++;
                c--;
            } else if (*c == 'R') {
                c++;
                int seed = ::atoi(c);
                while (*c >= '0' && *c <= '9')
                    c++;
                c--;
                cout << "random seed: " << seed << endl;
                srand(seed);
            } else if (*c == 'S') {
                const char *arg = c;
                c++;
                g_compose_strategy = *c++ - '1';
                if (g_compose_strategy < 0 ||
                   g_compose_strategy >= MAX_COMPOSE_STRATEGY) {
                    cerr << "Unknown option: " << arg << endl;
                    return 2;
                }
                g_collapse_strategy = *c++ - '1';
                if (g_collapse_strategy < 0 ||
                   g_collapse_strategy >= MAX_COLLAPSE_STRATEGY) {
                    cerr << "Unknown option: " << arg << endl;
                    return 2;
                }
                if (*c == '1' || *c == '2') {
                    if (*c == '2')
                        g_merge_and_shrink_bound_is_for_product = false;
                    c++;
                }
                c--;
            } else {
                cerr << "Unknown option: " << *c << endl;
                return 2;
            }
        }
    }

    if (fd_heuristic) {
        cout << "Composition strategy: ";
        if (g_compose_strategy == COMPOSE_LINEAR_CG_GOAL_LEVEL) {
            cout << "linear CG/GOAL, tie breaking on level (main)";
        } else if (g_compose_strategy == COMPOSE_LINEAR_CG_GOAL_RANDOM) {
            cout << "linear CG/GOAL, tie breaking random";
        } else if (g_compose_strategy == COMPOSE_LINEAR_GOAL_CG_LEVEL) {
            cout << "linear GOAL/CG, tie breaking on level";
        } else if (g_compose_strategy == COMPOSE_LINEAR_RANDOM) {
            cout << "linear random";
        } else if (g_compose_strategy == COMPOSE_DFP) {
            cout << "Draeger/Finkbeiner/Podelski";
        }
        cout << endl;
        if (g_compose_strategy == COMPOSE_DFP) {
            cerr << "DFP composition strategy not implemented." << endl;
            return 2;
        }

        cout << "Collapsing strategy: ";
        if (g_collapse_strategy == COLLAPSE_HIGH_F_LOW_H) {
            cout << "high f/low h (main)";
        } else if (g_collapse_strategy == COLLAPSE_LOW_F_LOW_H) {
            cout << "low f/low h";
        } else if (g_collapse_strategy == COLLAPSE_HIGH_F_HIGH_H) {
            cout << "high f/high h";
        } else if (g_collapse_strategy == COLLAPSE_RANDOM) {
            cout << "random states";
        } else if (g_collapse_strategy == COLLAPSE_DFP) {
            cout << "Draeger/Finkbeiner/Podelski";
        }
        cout << endl;
    }

    if (!cg_heuristic && !cyclic_cg_heuristic
       && !ff_heuristic && !additive_heuristic && !goal_count_heuristic
       && !blind_search_heuristic && !fd_heuristic && !hsp_max_heuristic
       && !lm_cut_heuristic && !lm_heuristic && !use_hm && !use_selective_max
       && !SP_heuristic && !lm_enriched_heuristic && !maximum_heuristic) {
        cerr << "Error: you must select at least one heuristic!" << endl
             << "If you are unsure, choose options \"cCfF\"." << endl;
        return 2;
    }

    //istream &in = cin;
	const char* input_filename = "output";
    ifstream in(input_filename);

    in >> poly_time_method;
    if (poly_time_method) {
        cout << "Poly-time method not implemented in this branch." << endl;
        cout << "Starting normal solver." << endl;
    }

    read_everything(in);
    // dump_everything();


    //cout << "Generating landmarks" << endl;
    //LandmarksGraph *lm_graph = new LandmarksGraphNew(true, true, false, false);
    //LandmarksGraph *lm_graph = new LandmarksGraphNew(new Exploration);
    //lm_graph->read_external_inconsistencies();
    //lm_graph->generate();
    //cout << "Generated " << lm_graph->number_of_landmarks() << " landmarks and "
    //<< lm_graph->number_of_edges() << " orderings" << endl;


    int iteration_no = 0;
    bool solution_found = false;
    int wa_star_weights[] = {10, 5, 3, 2, 1, -1};
    int wastar_bound = numeric_limits<int>::max();
    int wastar_weight = wa_star_weights[0];
    bool reducing_weight = true;
    bool synergy = false;

    Heuristic* sph = build_sp_heuristic(argv[2], NULL);
    do {
        iteration_no++;
        cout << "Search iteration " << iteration_no << endl;
        if (reducing_weight && wa_star_weights[iteration_no - 1] != -1) {
            wastar_weight = wa_star_weights[iteration_no - 1];
        } else {
            cout << "No more new weight, weight is " << wastar_weight << endl;
            reducing_weight = false;
        }

        SearchEngine *engine = 0;
        if (a_star_search) {
            engine = new AStarSearchEngine;
        } else if (use_gen_search) {
            engine = new EagerGreedyBestFirstSearchEngine;
        } else if (use_lazy_search) {
            engine = new LazyBestFirstSearchEngine();
        } else if (use_wa_star) {
            engine = new LazyWeightedAStar(weight);
        } else if (iterative_search) {
            engine = new LazyWeightedAStar(wastar_weight);
            ((LazyWeightedAStar*)engine)->set_bound(wastar_bound);
        } else if (use_ehc_search) {
            engine = new EnforcedHillClimbingSearch();
            if (ehc_rank_by_preferred) {
                ((EnforcedHillClimbingSearch*)engine)->set_preferred_usage(rank_preferred_first);
            }
        } else {
            cerr << "Must select search algorithm" << endl;
            exit(3);
        }


        // Test if synergies can be used between FF heuristic and landmark pref. ops.
        // Used to achieve LAMA's behaviour. (Note: this uses a different version
        // of the FF heuristic than if the FF heuristic is run by itself
        // or in other combinations.)
        if ((ff_heuristic || ff_preferred_operators) && lm_preferred) {
            LamaFFSynergy *lama_ff_synergy = new LamaFFSynergy(
                lm_preferred, lm_heuristic_admissible, lm_heuristic_optimal, lm_type);
            engine->add_heuristic(
                lama_ff_synergy->get_lama_heuristic_proxy(),
                lm_heuristic, lm_preferred);
            engine->add_heuristic(
                lama_ff_synergy->get_ff_heuristic_proxy(),
                ff_heuristic, ff_preferred_operators);
            synergy = true;
        }

        if (cg_heuristic || cg_preferred_operators)
            engine->add_heuristic(new CGHeuristic, cg_heuristic,
                                  cg_preferred_operators);
        if (cyclic_cg_heuristic || cyclic_cg_preferred_operators)
            engine->add_heuristic(new CyclicCGHeuristic, cyclic_cg_heuristic,
                                  cyclic_cg_preferred_operators);
        if (additive_heuristic || additive_preferred_operators)
            engine->add_heuristic(new AdditiveHeuristic, additive_heuristic,
                                  additive_preferred_operators);
        if ((ff_heuristic || ff_preferred_operators) && !synergy)
            engine->add_heuristic(new FFHeuristic, ff_heuristic,
                                  ff_preferred_operators);
        if (goal_count_heuristic)
            engine->add_heuristic(new GoalCountHeuristic, true, false);
        if (blind_search_heuristic)
            engine->add_heuristic(new BlindSearchHeuristic, true, false);
        if (fd_heuristic)
            engine->add_heuristic(new FinkbeinerDraegerHeuristic, true, false);
        if (hsp_max_heuristic)
            engine->add_heuristic(new HSPMaxHeuristic, true, false);
        if (lm_cut_heuristic)
            engine->add_heuristic(new LandmarkCutHeuristic, true, false);
        if (SP_heuristic) {
        	engine->add_heuristic(sph, true, false);
//        	engine->add_heuristic(build_sp_heuristic(argv[argc-1], NULL), true, false);
        }
        if (maximum_heuristic) {
#ifdef USE_LP
        	engine->add_heuristic(
        	        new SelectivePartitionHeuristic(num_clusters,
        	                argv[argc-1],
        	                sample_mode,
        	                partition,
        	                test_mode,
        	                use_initial_optimal,
        	                use_state_var_features,
        	                use_uniform_sp_features),
        	        true, false);
#else
		cout << "No LP Solver defined in this version" << endl;
		exit(1);
#endif
        }
        if (lm_enriched_heuristic)
        	engine->add_heuristic(
        	        build_lm_enriched_heuristic(argv[argc-1], NULL, lm_enriched_heuristic_type),
        	        true, false);

        if (use_hm) {
            cout << "Using h^" << m_hm << endl;
            engine->add_heuristic(new HMHeuristic(m_hm), true, false);
        }
        if (lm_heuristic && !synergy) {
            engine->add_heuristic(
                new LandmarksCountHeuristic(lm_preferred, lm_heuristic_admissible, lm_heuristic_optimal, lm_type),
                true, lm_preferred);
        }
        if (use_selective_max) {
            SelectiveMaxHeuristic *sel_max = new SelectiveMaxHeuristic();
            sel_max->add_heuristic(new LandmarksCountHeuristic(lm_preferred, lm_heuristic_admissible, lm_heuristic_optimal, lm_type));
            sel_max->add_heuristic(new LandmarkCutHeuristic);
            engine->add_heuristic(sel_max, true, false);
        }

        Timer search_timer;
        engine->search();
        search_timer.stop();
        g_timer.stop();
        if (engine->found_solution()) {
//            int plan_cost = save_plan(engine->get_plan());
    	    int plan_cost = save_plan(engine->get_plan(), plan_filename);
            wastar_bound = plan_cost;
        } else {
            iterative_search = false;
        }

        engine->statistics();

        if (cg_heuristic || cg_preferred_operators) {
            cout << "Cache hits: " << g_cache_hits << endl;
            cout << "Cache misses: " << g_cache_misses << endl;
        }
        cout << "Search time: " << search_timer << endl;
        cout << "Total time: " << g_timer << endl;

        solution_found = engine->found_solution();

        delete engine;
    } while (iterative_search);

}
/*
int save_plan(const vector<const Operator *> &plan) {
    ofstream outfile;
    int plan_cost = 0;
    outfile.open("sas_plan", ios::out);
    for (int i = 0; i < plan.size(); i++) {
        cout << plan[i]->get_name() << " (" << plan[i]->get_cost() << ")" << endl;
        outfile << "(" << plan[i]->get_name() << ")" << endl;
        plan_cost += plan[i]->get_cost();
    }
    outfile.close();
    cout << "Plan length: " << plan.size() << " step(s)." << endl;
    cout << "Plan cost: " << plan_cost << endl;
    return plan_cost;
}
*/
int save_plan(const vector<const Operator *> &plan, const string& filename) {
    ofstream outfile;
    int plan_cost = 0;
//    outfile.open("sas_plan", ios::out);
	outfile.open(filename.c_str(), ios::out);
   for (int i = 0; i < plan.size(); i++) {
        cout << plan[i]->get_name() << " (" << plan[i]->get_cost() << ")" << endl;
        outfile << "(" << plan[i]->get_name() << ")" << endl;
        plan_cost += plan[i]->get_cost();
    }
    outfile.close();
    cout << "Plan length: " << plan.size() << " step(s)." << endl;
    cout << "Plan cost: " << plan_cost << endl;
    return plan_cost;
}
