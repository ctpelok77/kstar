#include "plan_reconstructor.h"
#include "util.h"

#include <dirent.h>
#include <sys/stat.h>
#include <fstream>
#include <iostream>
#include <sstream>
#include "../successor_generator.h"
#include "../globals.h"

namespace kstar {

PlanReconstructor::PlanReconstructor(std::unordered_map<Node, Node>& parent_sap,
                                      std::unordered_set<Edge>& cross_edge,
                                      StateID goal_state,
                                      StateRegistry* state_registry,
                                      SearchSpace* search_space,
                                      bool skip_reorderings,     
                                      Verbosity verbosity) :
                                              parent_node(parent_sap),
                                              cross_edge(cross_edge),
                                              goal_state(goal_state),
                                              state_registry(state_registry),
                                              search_space(search_space),
                                              skip_reorderings(skip_reorderings),
                                              verbosity(verbosity) , attempted_plans(0) {
}

void PlanReconstructor::clear() {
    accepted_plans.clear();
    attempted_plans = 0;
}

void PlanReconstructor::set_goal_state(StateID goal_state) {
    this->goal_state = goal_state;
}

std::vector<Node> PlanReconstructor::djkstra_traceback(Node node)    {
    vector<Node> path;
    Node current_sap = node;
    for(;;) {
        path.push_back(current_sap);
        if(!current_sap.sap->op) {
            break;
        }
        current_sap = parent_node.at(current_sap);
    }
    reverse(path.begin(), path.end());
    return path;
}

vector<Node> PlanReconstructor::compute_sidetrack_seq(vector<Node>& path) {
    vector<Node> seq;
    int last_index = path.size() - 1;
    Node last_element = path[last_index];
    seq.push_back(last_element);

    for (size_t i = last_index; i >= 2; --i) {
       Edge e(path[i-1], path[i]);

        if(cross_edge.find(e) != cross_edge.end()) {
            seq.push_back(path[i - 1]);
        }
    }
    reverse(seq.begin(), seq.end());
    return seq;

}

void PlanReconstructor::extract_plan(vector<Node> &seq,
                                    Plan &plan,
                                    StateSequence &state_seq) {

    GlobalState current_state = state_registry->lookup_state(goal_state);
    state_seq.push_back(current_state.get_id());
    int seq_index = 0;
    int seq_size = seq.size();
    for(;;) {
        const SearchNodeInfo &info = search_space->search_node_infos[current_state];
        // Initial state reached and all edges of seq consumed
        if (info.creating_operator == -1 && seq_index == seq_size) {
            assert(info.parent_state_id == StateID::no_state);
            break;
        }

        // second last edge in seq and attachable to what we already have
        if(seq_index <= seq_size - 1 && seq[seq_index].sap->to == current_state.get_id()) {
            // prepend edge from seq
            plan.push_back(seq[seq_index].sap->op);
            current_state = state_registry->lookup_state(seq[seq_index].sap->from);
            ++seq_index;
        }
        else {
            // prepend tree edge
            const GlobalOperator *op = &g_operators[info.creating_operator];
            plan.push_back(op);
            current_state = state_registry->lookup_state(info.parent_state_id);
        }
        state_seq.push_back(current_state.get_id());
        if (verbosity >= Verbosity::NORMAL) {
            // Check that the last op on plan leads from the last state in state_seq to the one before it
            StateID parentID = state_seq[state_seq.size()-1]; 
            StateID childID = state_seq[state_seq.size()-2]; 
            const GlobalOperator* curr_op = plan[plan.size() - 1];
            GlobalState parent = state_registry->lookup_state(parentID);
            GlobalState next = state_registry->get_successor_state(parent, *curr_op);
            if (next.get_id() != childID) {
                cout << "----------------------" << endl;
                cout << "State in sequence" << endl;
                GlobalState child = state_registry->lookup_state(childID);
                child.dump_fdr();
                cout << "Real successor" << endl;
                next.dump_fdr();
                cout << "----------------------" << endl;
            }
        }

    }
    reverse(plan.begin(), plan.end());
    reverse(state_seq.begin(), state_seq.end());
}

bool PlanReconstructor::is_simple_plan(StateSequence seq, StateRegistry* state_registry) {
    PerStateInformation<bool> seen;
    for (size_t i = 0; i < seq.size(); ++i) {
        GlobalState s = state_registry->lookup_state(seq[i]);
        if (!seen[s]) {
           seen[s] = true;
        }
        else {
           return false;
        }
    }
    return true;
}

bool PlanReconstructor::add_plan(Node node,
                                 std::vector<Plan>& plans,
                                 std::vector<StateSequence>& state_sequences,
                                 bool simple_plans_only) {
    // Returns a boolean whether the plan was added
    attempted_plans++;
    if (attempted_plans % 100000 == 0) {
        printf ("Attempted plans: %4.1fM\n", attempted_plans / 1000000.0);
    }
    vector<Node> path = djkstra_traceback(node);
    vector<Node> seq;

    if (path.size() > 1) {
         seq = compute_sidetrack_seq(path);
    }

    Plan plan;
    StateSequence state_seq;
    extract_plan(seq, plan, state_seq);
    plan.shrink_to_fit();
    plan.pop_back();

    if (!simple_plans_only || is_simple_plan(state_seq, state_registry)) {
        if (!is_duplicate(plan)) {
            plans.push_back(plan);
            state_seq.shrink_to_fit();
            state_seq.pop_back();
            state_sequences.push_back(state_seq);
            return true;
        }
    }
    return false;
}

bool PlanReconstructor::is_duplicate(const Plan& plan) {
    if (!skip_reorderings)
        return false;
    
    // Checks whether the plan is a duplicate of an existing plan, and if not, add it to existing plans

    // Each plan is kept as a sorted vector by operator ids
    Plan unordered_plan = plan;
    std::sort(unordered_plan.begin(), unordered_plan.end());
	std::pair<PlansSet::iterator, bool > result = accepted_plans.insert(unordered_plan);
    return !result.second;
}

void PlanReconstructor::save_plans(std::vector<Plan>& plans, bool dump_plans) {
    for (auto& plan : plans) {
        if (dump_plans)
            dump_dot_plan(plan);
        save_plan(plan, true);
    }
}

void PlanReconstructor::preprocess_and_dump_state_action_pairs_to_json(std::vector<Plan>& plans, std::vector<StateSequence>& state_sequences, std::string file_name) {
    if (state_sequences.size() == 0)
        return;
    std::unordered_map<StateID, OperatorSet> ops_by_state; 
    for (size_t i = 0; i< plans.size(); ++i) {
        const Plan& plan = plans[i];
        const StateSequence& states = state_sequences[i];
        if (verbosity >= Verbosity::NORMAL) {
            if (plan.size() + 1 != states.size()) {
                cout << "PROBLEM, the action sequence and state sequence sizes do not match! " << plan.size() << ", " << states.size() << endl;
                cout << "----------------------" << endl;
                for (size_t j = 0; j< states.size(); ++j) {
                    GlobalState current_state = state_registry->lookup_state(states[j]);
                    current_state.dump_fdr();
                    cout << endl;
                    if (j < plan.size()) {
                        plan[j]->dump();
                    }
                    cout << "----------------------" << endl;
                }
            }
        }
        for (size_t j = 0; j< plan.size(); ++j) {
            ops_by_state[states[j]].insert(plan[j]);
        }
    }
    StateID init = state_sequences[0][0];
    ofstream os(file_name);
    os << "{ ";
    os << "\"fd_initial\" : ";
    dump_state_json(init, os);
    os << "," << endl;
    os << "\"data\" : [" << endl;
    bool first = true;
    for (auto elem : ops_by_state) {
        if (!first) {
            os << "," << endl;
        }
        first = false;
        dump_state_with_actions(elem.first, elem.second, os);
    }
    os << "]";
    os << "}" << endl;
}


void PlanReconstructor::dump_dot_plan(const Plan& plan) {
    stringstream node_stream, edge_stream;
    node_stream << "digraph {\n";
    GlobalState current = state_registry->get_initial_state();
    node_stream << current.get_id().hash() << " [ peripheries=\"1\", shape=\"rectangle\", ";
    if (test_goal_original(current)) {
        node_stream << "style=\"rounded, filled\", fillcolor=\"red\", ";
    } else {
        node_stream << "style=\"rounded, filled\", fillcolor=\"yellow\", ";
    }
    node_stream << "label=\"#"<< current.get_id().hash() << "\\n" << "s="<< state_label(current) << "\\n";
    node_stream << "\" ]\n";

    for (size_t i = 0; i < plan.size(); ++i) {
        const GlobalOperator* op = plan[i];
        GlobalState next = state_registry->get_successor_state(current, *op);

        node_stream << next.get_id().hash() << " [ peripheries=\"1\", shape=\"rectangle\", ";
        if (test_goal_original(next)) {
            node_stream << "style=\"rounded, filled\", fillcolor=\"red\", ";
        } else {
            node_stream << "style=\"rounded, filled\", fillcolor=\"yellow\", ";
        }
        node_stream << "label=\"#"<< next.get_id().hash() << "\\n" << "s="<< state_label(next) << "\\n";
        node_stream << "\" ]\n";

        edge_stream << current.get_id().hash() << "  ->  " << next.get_id().hash();
        edge_stream <<" [ label=\"" << op->get_name();
        edge_stream << "/"<< op->get_cost();
        edge_stream << "\"";
        edge_stream << " ]\n";
        current = next;

    }
    edge_stream << "}\n";

    ostringstream filename;
    filename << g_plan_filename;
    int plan_number = g_num_previously_generated_plans + 1;

    filename << "." << plan_number << ".dot";

    // Write state space to dot file
    ofstream file(filename.str());

    file << node_stream.rdbuf();
    file << edge_stream.rdbuf();
    file.close();
}

void PlanReconstructor::dump_state_with_actions(const StateID& stateID, const OperatorSet& actions, std::ostream& os) {

    os << "{ ";
    os << "\"state\" : ";
    dump_state_json(stateID, os);
    os << ", ";
    os << "\"actions_from_plans\" : " ;
    os << "[";
    bool first = true;
    for (auto op : actions) {
        if (!first) {
            os << "," << endl;
        }
        first = false;
        dump_action_json(op, os);

    }
    os << "]";
    os << ",";

    GlobalState current_state = state_registry->lookup_state(stateID);
    vector<const GlobalOperator *> applicable_ops;
    g_successor_generator->generate_applicable_ops(current_state, applicable_ops);

    vector<const GlobalOperator *> other_applicable_ops;
    for (auto op : applicable_ops) {
        if (actions.find(op) != actions.end())
            other_applicable_ops.push_back(op);
    }

    os << "\"other_applicable_actions\" : " ;
    os << "[";
    first = true;
    for (auto op : other_applicable_ops) {
        if (!first) {
            os << "," << endl;
        }
        first = false;
        dump_action_json(op, os);
    }
    os << "]";
    os << "}";
}

void PlanReconstructor::action_name_parsing(std::string op_name, std::vector<std::string>& parsed) {
    std::string s = op_name.substr(1, op_name.length() - 2); // without the first and last
    size_t pos = 0;
    while ((pos = s.find(" ")) != std::string::npos) {
        parsed.push_back(s.substr(0, pos));
        s.erase(0, pos + 1);
    }
    parsed.push_back(s);
}


void PlanReconstructor::dump_action_json(const GlobalOperator *op, std::ostream& os) {
/*
{"params": ["ca", "d2", "c", "n1", "c0", "n0"], "name": "sendtohome"}
*/
    std::vector<std::string> parsed;
    action_name_parsing(op->get_name(), parsed);

    os << "{ \"name\" : " << "\"" << parsed[0] << "\"" ;
    os << ", ";
    os << "\"params\" : [";
    bool first = true;
    for (size_t i = 1; i< parsed.size(); ++i) {
        if (!first) {
            os << ", ";
        }
        first = false;
        os << "\"" << parsed[i] << "\"";
    }
    os << "]}";
}

std::string PlanReconstructor::fact_to_pddl(std::string fact) const {
    /*  "Atom bottomcol(ha)" -> "(bottomcol ha)""
        "NegatedAtom clear(da)" -> "(not (clear da))"
    */
    // Check if starts with "Atom"
    if (fact.substr(0,4) == "Atom") {
        return restructure_fact(fact.substr(5, fact.length()-5));
    }   
    if (fact.substr(0,11) == "NegatedAtom") {
        return "(not " + restructure_fact(fact.substr(12, fact.length()-12)) + ")";
    }   
    cerr << "SHOULD NOT HAPPEN!!! Got fact not starting with Atom or NegatedAtom: " << fact << endl;
    return fact;
}

std::string PlanReconstructor::restructure_fact(std::string fact) const {
    /*  "bottomcol(ha)" -> "(bottomcol ha)""
        "at(da db)" -> "(at da db)"
    */
    size_t pos = fact.find("(");
    std::string name = fact.substr(0, pos);
    return "(" + name + " " + fact.substr(pos+1, fact.length() - pos - 1);
}


void PlanReconstructor::dump_state_json(const StateID& state, std::ostream& os) {
/*
["(value c0 n0)", "(value ca n1)", "(value c2 n2)", "(value c3 n3)", "(value c4 n4)"]
*/
    GlobalState current_state = state_registry->lookup_state(state);
    State s(state_registry->get_task(), current_state.get_values());
    std::vector<std::string> state_parsed;

    for (FactProxy fact : s) {
        string fact_name = fact.get_name();
        if (fact_name != "<none of those>" && fact_name != "false" && fact_name != "true") {
            state_parsed.push_back(fact_to_pddl(fact_name));
        }
    }

    os << "[";
    bool first = true;
    for (size_t i = 0; i< state_parsed.size(); ++i) {
        if (!first) {
            os << ", ";
        }
        first = false;
        os << "\"" << state_parsed[i] << "\"";
    }
    os << "]";
}


}
