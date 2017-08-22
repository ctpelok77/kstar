#include "util.h"

namespace kstar {

    bool is_self_loop(SearchNode node, SearchNode succ_node) {
        if (node.get_state_id() == succ_node.get_state_id())
            return true;
        return false;
    }

    int get_cost_heap_edge(Sap &from, Sap &to) {
        assert(to->get_delta() - from->get_delta() >= 0);
        int cost_heap_edge = to->get_delta() - from->get_delta();
        return cost_heap_edge;
    }

    int get_cost_cross_edge(Sap &to) {
        int cost_cross_edge = to->get_delta();
        return cost_cross_edge;
    }

    void print_tree_heap(vector<Sap> &v) {
        for (size_t i = 0; i < v.size(); ++i) {
            Sap sap = v[i];
            cout << "(" << sap->get_from_state().get_state_tuple() << ","
                 << sap->get_to_state().get_state_tuple() << ")" << endl;
        }
    }

    std::string get_node_name(Node &p, StateRegistry *state_registry) {
        std::string str;
        if (p.sap->from == StateID::no_state) {
            return "R";
        }

        GlobalState s = p.sap->get_from_state();
        GlobalState succ = p.sap->get_to_state();
        GlobalState heap_s = state_registry->lookup_state(p.heap_state);


        str = "(" + s.get_state_tuple() + ","\
 + succ.get_state_tuple() + ") tree_heap[" + heap_s.get_state_tuple() + "]";
        return str;
    }

    void notify_generate(Node &p, StateRegistry *state_registry) {
        std::string node_name = get_node_name(p, state_registry);
        std::cout << "Generating node " << node_name << " from queue" << std::endl;
    }

    void notify_push(Node &node, StateRegistry *state_registry) {
        std::string node_name = get_node_name(node, state_registry);
        cout << "Pushing " << node_name << " " << node.sap->op->get_name()
             << " g= " << node.g << " to queue" << endl;
    }

    void notify_cross_edge(Node &node, StateRegistry *state_registry) {
        std::string node_name = get_node_name(node, state_registry);
        cout << "Pushing cross_edge" << node_name << " " << node.sap->op->get_name()
             << " g= " << node.g << " to queue" << endl;
    }

    void notify_inheap_edge(Node &node, StateRegistry *state_registry) {
        std::string node_name = get_node_name(node, state_registry);
        cout << "Pushing inheap_edge" << node_name << " " << node.sap->op->get_name()
             << " g= " << node.g << " to queue" << endl;
    }

    void notify_tree_heap_edge(Node &node, StateRegistry *state_registry) {
        std::string node_name = get_node_name(node, state_registry);
        cout << "Pushing tree_heap edge" << node_name << " " << node.sap->op->get_name()
             << " g= " << node.g << " to queue" << endl;
    }

    void notify_expand(Node &p, StateRegistry *state_registry, int &num_node_expansions) {
        std::string node_name = get_node_name(p, state_registry);

        if (node_name != "R") {
            std::string op = p.sap->op->get_name();
            std::cout << num_node_expansions << ". " << "Expanding node " << node_name << " "
                      << op << " g=" << p.g << " " << p.sap->op->get_name() << std::flush << std::endl;
        } else {
            std::cout << num_node_expansions << ". " << "Expanding node "
                      << node_name << " g=" << p.g << std::flush << std::endl;
        }
        ++num_node_expansions;
    }

}
