#ifndef KSTAR_UTIL
#define KSTAR_UTIL

#include "../search_space.h"
#include "kstar_types.h"
#include "../state_action_pair.h"

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

}
#endif
