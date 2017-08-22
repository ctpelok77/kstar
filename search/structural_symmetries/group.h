#ifndef STRUCTURAL_SYMMETRIES_GROUP_H
#define STRUCTURAL_SYMMETRIES_GROUP_H

#include "operator_permutation.h"

#include <memory>
#include <vector>

class GlobalState;
class StateRegistry;

//namespace structural_symmetries {
class GraphCreator;
class Permutation;
//class OperatorPermutation;
//}

namespace options {
class Options;
}

namespace utils {
class RandomNumberGenerator;
}

//namespace structural_symmetries {

enum class SearchSymmetries {
    NONE,
    OSS,
    DKS
};
enum class SymmetricalLookups {
    NONE,
    ONE_STATE,
    SUBSET_OF_STATES,
    ALL_STATES
};

class Group {

    // Options for the type of symmetries used
    bool stabilize_initial_state;
    SearchSymmetries search_symmetries;
    SymmetricalLookups symmetrical_lookups;
    int rw_length_or_number_symmetric_states;
    const std::shared_ptr<utils::RandomNumberGenerator> rng;
    bool dump;
    bool keep_operator_symmetries;
    bool initialized;
    GraphCreator *graph_creator;
    std::vector<const Permutation *> generators;
    std::vector<const OperatorPermutation *> operator_generators;

    // Methods for SL
    void compute_random_symmetric_state(const GlobalState &state,
                                        StateRegistry &symmetric_states_registry,
                                        std::vector<GlobalState> &states) const;
    void compute_subset_all_symmetric_states(const GlobalState &state,
                                             StateRegistry &symmetric_states_registry,
                                             std::vector<GlobalState> &states) const;

    // Methods for OSS
    typedef std::vector<short int> Trace;
    void get_trace(const GlobalState& state, Trace& full_trace) const;
    Permutation *compose_permutation(const Trace &) const;

    void delete_generators();
    const Permutation &get_permutation(int index) const;
public:
    explicit Group(const options::Options &opts);
    ~Group();
    static int num_identity_generators;
    void compute_symmetries();

    static void add_permutation(void*, unsigned int, const unsigned int *);
    void add_generator(const Permutation *gen);
    void add_operator_generator(const unsigned int * full_perm);
    int get_num_generators() const;
    void dump_generators() const;
    void dump_variables_equivalence_classes() const;
    void statistics() const;
    bool is_stabilizing_initial_state() const {
        return stabilize_initial_state;
    }
    bool is_initialized() const {
        return initialized;
    }
    bool has_symmetries() const {
        return !generators.empty();
    }
    SearchSymmetries get_search_symmetries() const {
        return search_symmetries;
    }
    SymmetricalLookups get_symmetrical_lookups() const {
        return symmetrical_lookups;
    }

    // Methods used for symmetric lookups
    void compute_symmetric_states(const GlobalState &state,
                                  StateRegistry &symmetric_states_registry,
                                  std::vector<GlobalState> &states) const;

    // Used for OSS
    int *get_canonical_representative(const GlobalState &state) const;
    // Used for path tracing (OSS and DKS)
    Permutation *create_permutation_from_state_to_state(
        const GlobalState &from_state, const GlobalState &to_state) const;
};
//}
#endif
