#ifndef KSTAR_SUCCESSOR_GENERATOR_H
#define KSTAR_SUCCESSOR_GENERATOR_H

#include "kstar_types.h"

namespace kstar {

class SuccessorGenerator {
    PerStateInformation<vector<Sap>> &tree_heap;
    PerStateInformation<vector<Sap>> &incomming_heap;
    std::unordered_map<Node, Node> &parent_node;
    std::unordered_set<Edge> &cross_edge;
    StateRegistry* state_registry;

public:
    SuccessorGenerator(PerStateInformation<vector<Sap>> &tree_heap,
                       PerStateInformation<vector<Sap>> &incoming_heap,
                       std::unordered_map<Node, Node> &parent_sap,
                       std::unordered_set<Edge> &cross_edge,
                       StateRegistry* state_registry);

    virtual ~SuccessorGenerator() = default;
    void get_successor_pg_root(shared_ptr<Node> pg_root,
                               Node &successor, bool successor_only = false);
    void set_parent(Node &node, Node succ_node, bool is_cross_edge);

    void get_successors(Node &sap, vector<Node> &successor_sap,
                        bool successors_only = false);
    int get_max_successor_delta(Node& sap, shared_ptr<Node> pg_root);
    void add_cross_edge(Node &p, vector<Node> &successors,
                        bool successors_only = false);
       void add_inheap_successors(Node &node, vector<Node> &successors,
                               bool successor_only = false);
    void add_treeheap_successors(Node &node, vector<Node> &successors,
                               bool successor_only = false);
    bool is_inheap_top(Node &node);
    bool is_treeheap_top(Node &node);
};
}
#endif
