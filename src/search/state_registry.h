#ifndef STATE_REGISTRY_H
#define STATE_REGISTRY_H

#include "abstract_task.h"
#include "axioms.h"
#include "global_state.h"
#include "state_id.h"

#include "algorithms/int_packer.h"
#include "algorithms/segmented_vector.h"
#include "structural_symmetries/group.h"

#include "utils/hash.h"

#include <set>
#include <unordered_set>

/*
  Overview of classes relevant to storing and working with registered states.

  GlobalState
    This class is used for manipulating states.
    It contains the (uncompressed) variable values for fast access by the heuristic.
    A State is always registered in a StateRegistry and has a valid ID.
    States can be constructed from a StateRegistry by factory methods for the
    initial state and successor states.
    They never own the actual state data which is borrowed from the StateRegistry
    that created them.

  StateID
    StateIDs identify states within a state registry.
    If the registry is known, the ID is sufficient to look up the state, which
    is why ids are intended for long term storage (e.g. in open lists).
    Internally, a StateID is just an integer, so it is cheap to store and copy.

  PackedStateBin (currently the same as unsigned int)
    The actual state data is internally represented as a PackedStateBin array.
    Each PackedStateBin can contain the values of multiple variables.
    To minimize allocation overhead, the implementation stores the data of many
    such states in a single large array (see SegmentedArrayVector).
    PackedStateBin arrays are never manipulated directly but through
    a global IntPacker object.

  -------------

  StateRegistry
    The StateRegistry allows to create states giving them an ID. IDs from
    different state registries must not be mixed.
    The StateRegistry also stores the actual state data in a memory friendly way.
    It uses the following class:

  SegmentedArrayVector<PackedStateBin>
    This class is used to store the actual (packed) state data for all states
    while avoiding dynamically allocating each state individually.
    The index within this vector corresponds to the ID of the state.

  PerStateInformation<T>
    Associates a value of type T with every state in a given StateRegistry.
    Can be thought of as a very compactly implemented map from GlobalState to T.
    References stay valid forever. Memory usage is essentially the same as a
    vector<T> whose size is the number of states in the registry.


  ---------------
  Usage example 1
  ---------------
  Problem:
    A search node contains a state together with some information about how this
    state was reached and the status of the node. The state data is already
    stored and should not be duplicated. Open lists should in theory store search
    nodes but we want to keep the amount of data stored in the open list to a
    minimum.

  Solution:

    SearchNodeInfo
      Remaining part of a search node besides the state that needs to be stored.

    SearchNode
      A SearchNode combines a StateID, a reference to a SearchNodeInfo and
      OperatorCost. It is generated for easier access and not intended for long
      term storage. The state data is only stored once an can be accessed
      through the StateID.

    SearchSpace
      The SearchSpace uses PerStateInformation<SearchNodeInfo> to map StateIDs to
      SearchNodeInfos. The open lists only have to store StateIDs which can be
      used to look up a search node in the SearchSpace on demand.

  ---------------
  Usage example 2
  ---------------
  Problem:
    In the LMcount heuristic each state should store which landmarks are
    already reached when this state is reached. This should only require
    additional memory, when the LMcount heuristic is used.

  Solution:
    The heuristic object uses a field of type PerStateInformation<std::vector<bool> >
    to store for each state and each landmark whether it was reached in this state.
*/

class Permutation;
class PerStateInformationBase;

class StateRegistry {
    struct StateIDSemanticHash {
        const segmented_vector::SegmentedArrayVector<PackedStateBin> &state_data_pool;
        int state_size;
        StateIDSemanticHash(
            const segmented_vector::SegmentedArrayVector<PackedStateBin> &state_data_pool,
            int state_size)
            : state_data_pool(state_data_pool),
              state_size(state_size) {
        }

        size_t operator()(StateID id) const {
            return utils::hash_sequence(state_data_pool[id.value],
                                        state_size);
        }
    };

    struct StateIDSemanticEqual {
        const segmented_vector::SegmentedArrayVector<PackedStateBin> &state_data_pool;
        int state_size;
        StateIDSemanticEqual(
            const segmented_vector::SegmentedArrayVector<PackedStateBin> &state_data_pool,
            int state_size)
            : state_data_pool(state_data_pool),
              state_size(state_size) {
        }

        bool operator()(StateID lhs, StateID rhs) const {
            const PackedStateBin *lhs_data = state_data_pool[lhs.value];
            const PackedStateBin *rhs_data = state_data_pool[rhs.value];
            return std::equal(lhs_data, lhs_data + state_size, rhs_data);
        }
    };

    /*
      Hash set of StateIDs used to detect states that are already registered in
      this registry and find their IDs. States are compared/hashed semantically,
      i.e. the actual state data is compared, not the memory location.
    */
    typedef std::unordered_set<StateID,
                               StateIDSemanticHash,
                               StateIDSemanticEqual> StateIDSet;

    /* TODO: The state registry still doesn't use the task interface completely.
             Fixing this is part of issue509. */
    /* TODO: AbstractTask is an implementation detail that is not supposed to
             leak. In the long run, we should store a TaskProxy here. */
    const AbstractTask &task;

    /* TODO: When we switch StateRegistry to the task interface, the next three
             members should come from the task. */
    const int_packer::IntPacker &state_packer;
    AxiomEvaluator &axiom_evaluator;
    const std::vector<int> &initial_state_data;
    const int num_variables;

    segmented_vector::SegmentedArrayVector<PackedStateBin> state_data_pool;
    // Used for DKS
    segmented_vector::SegmentedArrayVector<PackedStateBin> canonical_state_data_pool;
    StateIDSet registered_states;
    // Used for DKS
    StateIDSet canonical_registered_states;
    // Used for DKS
    std::shared_ptr<Group> group;

    GlobalState *cached_initial_state;
    mutable std::set<PerStateInformationBase *> subscribers;

    StateID insert_id_or_pop_state();
    // Used for DKS
    StateID insert_id_or_pop_state_dks();
    int get_bins_per_state() const;
public:
    StateRegistry(
        const AbstractTask &task, const int_packer::IntPacker &state_packer,
        AxiomEvaluator &axiom_evaluator, const std::vector<int> &initial_state_data);
    ~StateRegistry();

    // Used for DKS
    void set_group(const std::shared_ptr<Group> &group_) {
        group = group_;
    }

    /* TODO: Ideally, this should return a TaskProxy. (See comment above the
             declaration of task.) */
    const AbstractTask &get_task() const {
        return task;
    }

    int get_num_variables() const {
        return num_variables;
    }

    int get_state_value(const PackedStateBin *buffer, int var) const {
        return state_packer.get(buffer, var);
    }

    /*
      Returns the state that was registered at the given ID. The ID must refer
      to a state in this registry. Do not mix IDs from from different registries.
    */
    GlobalState lookup_state(StateID id) const;

    /*
      Returns a reference to the initial state and registers it if this was not
      done before. The result is cached internally so subsequent calls are cheap.
    */
    const GlobalState &get_initial_state();

    /*
      Returns the state that results from applying op to predecessor and
      registers it if this was not done before. This is an expensive operation
      as it includes duplicate checking.
    */
    GlobalState get_successor_state(const GlobalState &predecessor, const GlobalOperator &op);

    /*
      Registers and returns the state corresponding to the given state data.
      This is an expensive operation as it includes duplicate checking.
      Used for OSS.
    */
    GlobalState register_state_buffer(const int *state);

    /*
      Registers and returns the state corresponding to the given state.
      This is an expensive operation as it includes duplicate checking.
    */
    GlobalState copy_and_register_state(const GlobalState &state);

    /*
      Creates the permutation of the given state (which can be registered
      somewhere else). Registers and returns the permuted state if this was not
      done before. This is an expensive operation as it includes duplicate
      checking.
      Used for state tracing (OSS and DKS) and computing symmetric states (SL).
    */
    GlobalState permute_state(const GlobalState &state, const Permutation &permutation);

    /*
      Returns the number of states registered so far.
    */
    size_t size() const {
        if (group && group->has_symmetries() && group->get_search_symmetries() == SearchSymmetries::DKS)
            return canonical_registered_states.size();
        return registered_states.size();
    }

    int get_state_size_in_bytes() const;

    /*
      Remembers the given PerStateInformation. If this StateRegistry is
      destroyed, it notifies all subscribed PerStateInformation objects.
      The information stored in them that relates to states from this
      registry is then destroyed as well.
    */
    void subscribe(PerStateInformationBase *psi) const;
    void unsubscribe(PerStateInformationBase *psi) const;
};

#endif
