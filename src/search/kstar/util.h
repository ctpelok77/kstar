#ifndef KSTAR_UTIL_H
#define KSTAR_UTIL_H

#include "../search_space.h"
#include "kstar_types.h"
#include "../state_action_pair.h"
#include "../global_state.h"

namespace kstar {
    bool is_self_loop(SearchNode node, SearchNode succ_node);
    int get_cost_heap_edge(Sap &from, Sap &to);
    int get_cost_cross_edge(Sap &to);
    std::string get_node_name(Node& p, StateRegistry* state_registry);
    void notify_generate(Node& p, StateRegistry* state_registry);
    std::string get_node_name(Node& p, StateRegistry* state_registry);
    void notify_push(Node& p, StateRegistry* state_registry);
    void notify_cross_edge(Node& p, StateRegistry* state_registry);
    void notify_inheap_edge(Node& p, StateRegistry* state_registry);
    void notify_tree_heap_edge(Node& p, StateRegistry* state_registry);
    void notify_expand(Node& p, StateRegistry* state_registry, int &num_node_expansions);
    void print_tree_heap(vector<Sap>& v);
    void save_and_close(std::string filename, Stream &stream, Stream &node_stream);
    void add_dot_node(std::string id, std::string label, Stream &stream);

    std::string get_sap_id(Sap &sap, GlobalState s);
    std::string get_sap_label(Sap &sap);
    void begin_subgraph(std::string label, Stream &stream);
    void add_edge(std::string from_id, std::string to_id,
                  std::string label, Stream &stream);
    void print_node_sequence(std::vector<Node> &sequence, std::string name);
    void print_operator_sequence(Plan plan, std::string name);
}
#endif
