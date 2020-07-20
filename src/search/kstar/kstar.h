#ifndef KSTAR_KSTAR_H
#define KSTAR_KSTAR_H

#include "successor_generator.h"
#include "plan_reconstructor.h"
#include "kstar_types.h"

#include "../search_engines/top_k_eager_search.h"

#include <memory>

namespace kstar {

class KStar : public top_k_eager_search::TopKEagerSearch {
    void inc_optimal_plans_count(int plan_cost);
protected:
    int optimal_solution_cost;
    bool simple_plans_only;
    bool dump_states;
    bool dump_json;
    std::string json_filename;

    int num_node_expansions;
    bool djkstra_initialized;
    std::priority_queue<Node> queue_djkstra;
    std::unordered_map<Node, Node> parent_node;
    std::unordered_set<Edge> cross_edge;
    std::unique_ptr<PlanReconstructor> plan_reconstructor;
    std::shared_ptr<SuccessorGenerator> pg_succ_generator;
    // root of the path graph
    shared_ptr<Node> pg_root;
    void initialize_djkstra();
    // djkstra search return true if k solutions have been found and false otherwise
    bool djkstra_search();
    bool enough_nodes_expanded();
    void resume_astar();
    void init_tree_heaps(Node node);
    vector<Sap> djkstra_traceback(Node& top_pair);
    vector<Sap> compute_sidetrack_seq(Node& top_pair, vector<Sap>& path);
    void throw_everything();
    void add_plan(Node& p);
    bool enough_plans_found() const;
    bool enough_plans_found_topk() const;
    bool enough_plans_found_topq() const;
    void set_optimal_plan_cost(int plan_cost);
    void update_most_expensive_succ();
    void dump_tree_edge();
    void dump_path_graph();
    void dump_dot() const;
    virtual ~KStar() = default;
public:
    KStar (const options::Options &opts);
    void search() override;
};
}

#endif 
