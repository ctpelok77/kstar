#ifndef STATE_H
#define STATE_H

#include <iostream>
#include <vector>
using namespace std;

class Operator;

#include "state_var_t.h"
#include "state_handle.h"

class State {
    bool borrowed_buffer;
    // Values for vars. will later be converted to UnpackedStateData.
    state_var_t *vars;
    StateHandle handle;
    void copy_buffer_from(const state_var_t *buffer);
    // Only used for creating the initial state.
    explicit State(state_var_t *buffer);
    explicit State(const State &predecessor, const Operator &op);
public:
    explicit State(const StateHandle &handle);
    State(const State &state);
    State &operator=(const State &other);
    ~State();

    // TODO why is g_initial_state a pointer?
    static State *create_initial_state(state_var_t *initial_state_vars);
    static State create_registered_successor(const State &predecessor, const Operator &op);
    static State create_unregistered_successor(const State &predecessor, const Operator &op);

    int get_id() const;
    const StateHandle get_handle() const;

    int operator[](int index) const {
        return vars[index];
    }
    void dump() const;
    bool operator==(const State &other) const;
    bool operator<(const State &other) const;
    size_t hash() const;

    const state_var_t *get_buffer() const {
        return vars;
    }
};

#endif
