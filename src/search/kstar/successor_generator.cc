#include "successor_generator.h"
#include "util.h"

using namespace std;

namespace kstar {

SuccessorGenerator::SuccessorGenerator(PerStateInformation<vector<Sap>> &tree_heap,
                                       PerStateInformation<vector<Sap>> &incoming_heap,
                                          std::unordered_map<Node, Node> &parent_sap,
                                       std::unordered_set<Edge> &cross_edge,
                                       StateRegistry* state_registry) :
                                               tree_heap(tree_heap),
                                               incomming_heap(incoming_heap),
                                               parent_node(parent_sap),
                                               cross_edge(cross_edge),
                                               state_registry(state_registry) {
}

// For each node carrying an edge (u,v) we attach a pointer referring
// to R(u) = top node of H_T[u]
void SuccessorGenerator::add_cross_edge(Node &node,
                                        vector<Node> &successors,
                                        bool successors_only) {

    GlobalState u = state_registry->lookup_state(node.sap->from);
    if (tree_heap[u].empty())
        return;
    int succ_g = node.g + get_cost_cross_edge(tree_heap[u].front());
    Node succ_node(succ_g, tree_heap[u].front(), u.get_id());

    if (!successors_only) {
        succ_node.id = g_djkstra_nodes;
        ++g_djkstra_nodes;
        set_parent(node, succ_node, true);
    }

    successors.push_back(succ_node);
}

// Aljazzar and Leue: edge (u,v)
// root_in(v) keeps its only child from H_in (v).
void SuccessorGenerator::add_inheap_successors(Node &node,
                                               vector<Node> &successors,
                                               bool successors_only){
    GlobalState s  = node.sap->get_to_state();
    for (size_t i = 1; i < incomming_heap[s].size(); ++i) {
        Sap &sap = incomming_heap[s][i];
        int succ_g = node.g + get_cost_heap_edge(node.sap, sap);
        Node succ_node(succ_g, sap, node.heap_state);
        if (!successors_only) {
            succ_node.is_inheap_node = true;
            succ_node.id = g_djkstra_nodes;
            ++g_djkstra_nodes;
            set_parent(node, succ_node, false);
        }

        successors.push_back(succ_node);
    }
}

// Checks whether node is the root of the incomming
// heap for its corresponding heap_state
bool SuccessorGenerator::is_inheap_top(Node &node) {
    GlobalState s = node.sap->get_to_state();
    if (node.sap == incomming_heap[s].front())
        return true;
    return false;
}

// check whether node is the root of the treeheap
bool SuccessorGenerator::is_treeheap_top(Node &node) {
    GlobalState s = state_registry->lookup_state(node.heap_state);
    if (node.sap == tree_heap[s].front())
        return true;
    return false;
}

void SuccessorGenerator::add_treeheap_successors(Node &node,
                                                 vector<Node> &successors,
                                                 bool successors_only) {
    GlobalState s = state_registry->lookup_state(node.heap_state);
    for (size_t i = 1; i < tree_heap[s].size(); ++i) {
        Sap &sap = tree_heap[s][i];
        int succ_g = node.g + get_cost_heap_edge(node.sap, sap);
        Node succ_node(succ_g, sap, s.get_id());
        if (!successors_only) {
            succ_node.id = g_djkstra_nodes;
            ++g_djkstra_nodes;
            set_parent(node, succ_node, false);
        }

        successors.push_back(succ_node);
    }
}

void SuccessorGenerator::get_successors(Node &node,
                                        vector<Node> &successors,
                                        bool successors_only) {

    add_cross_edge(node, successors, successors_only);
    if (is_inheap_top(node)) {
        add_inheap_successors(node, successors, successors_only);
    }

    if (is_treeheap_top(node) && !node.is_inheap_node) {
        add_treeheap_successors(node, successors, successors_only);
    }
}


int SuccessorGenerator::get_max_successor_delta(Node& node,
                                                shared_ptr<Node> pg_root) {
    GlobalState s = state_registry->lookup_state(node.sap->to);
    int max = -1;
    vector<Node> successors;

    // is root node
    if(node.sap->from == StateID::no_state) {
        Node succ;
        get_successor_pg_root(pg_root, succ, true);
        successors.push_back(succ);
    }
    else {
        get_successors(node, successors, true);
    }

    for(auto succ : successors) {
        if (succ.sap->get_delta() > max) {
            max = succ.sap->get_delta();
        }
    }
    return max;
}

void SuccessorGenerator::set_parent(Node &node, Node succ_node, bool is_cross_edge) {
    parent_node.insert(pair<Node, Node>(succ_node, node));
    if (is_cross_edge) {
        Edge e(node, succ_node);
        cross_edge.insert(e);
    }
}

void SuccessorGenerator::get_successor_pg_root(shared_ptr<Node> pg_root,
                                               Node &successor, bool successor_only) {
    StateID goal_id = pg_root->sap->to;
    GlobalState goal_state = state_registry->lookup_state(goal_id);
    Sap succ_sap = tree_heap[goal_state].front();
    int succ_g = pg_root->g + get_cost_cross_edge(succ_sap);
    successor = Node(succ_g, succ_sap, goal_id);
    if (successor_only)
        return;
    successor.id = g_djkstra_nodes;

    set_parent(*pg_root, successor ,true);
    ++g_djkstra_nodes;
}
}
