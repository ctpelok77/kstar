#ifndef STATE_ACTION_PAIR
#define STATE_ACTION_PAIR 

//#include <boost/functional/hash.hpp>

#include "state_id.h"
#include "state_registry.h"
#include "search_space.h"
#include "global_operator.h"
#include "utils/util.h"
#include "utils/hash.h"

class StateActionPair {
public:
	StateID from;
	StateID to;
	const  GlobalOperator* op;
	const StateRegistry* reg;

	SearchSpace* ssp;
	static const StateActionPair no_sap; 

	StateActionPair(const StateActionPair& other)
			: from(other.from), to(other.to), op(other.op), reg(other.reg), ssp(other.ssp) {

	};

	StateActionPair(StateID from, StateID to, const GlobalOperator* op,
					const StateRegistry* reg, SearchSpace* ssp)
		: from(from), to(to), op(op), reg(reg), ssp(ssp) {

	};

	~StateActionPair() {
	}

	GlobalState get_from_state() {
		return reg->lookup_state(from);			
	};

	GlobalState get_to_state() {
		return reg->lookup_state(to);			
	};

	int from_g() const {
		const GlobalState from_state = reg->lookup_state(from);
		SearchNode from_node = ssp->get_node(from_state);
		return from_node.get_g();
	};

	int edge_cost() const {
		return op->get_cost();
	};	
	
	int to_g() const {
		const GlobalState to_state = reg->lookup_state(to);
		SearchNode to_node = ssp->get_node(to_state);
		return to_node.get_g();
	};

	int get_delta() const {
		if (from == StateID::no_state)
			return -1;
		const GlobalState to_state = reg->lookup_state(to);
		const GlobalState from_state = reg->lookup_state(from);
		SearchNode from_node = ssp->get_node(from_state);
		SearchNode to_node = ssp->get_node(to_state);

		return from_node.get_g() + op->get_cost() - to_node.get_g();
	};

	size_t hash() const {
		size_t value_from = from.hash();
		size_t value_to = to.hash();
		size_t seed = 0;
		utils::hash_combine(seed, value_from);
		utils::hash_combine(seed, value_to);
		return seed;
	}

	bool operator<(const StateActionPair &other) const {
		if (this->get_delta() < other.get_delta())
			return true;
		if (this->get_delta() > other.get_delta())
            return false;
		if (this->hash() < other.hash())
			return true;
		return false;
	};

	bool operator==(const StateActionPair& other) const {
		if (from != other.from)
			return false;
		if (to != other.to)
			return false;
		if (op->get_index() != other.op->get_index())
			return false;
		return true;
	};

	bool operator!=(const StateActionPair &other) const {
		if (*this == other)
			return false;
		return true;
	}
};
struct Node;

struct Node {
	int id = -1;
	int g = -1;
	shared_ptr<StateActionPair> sap = nullptr;
	StateID heap_state = StateID::no_state;
	bool is_inheap_node = false;

	Node() {
		id = -1;
		g = -1;
		sap = nullptr;
		heap_state = StateID::no_state;
	}

	Node (const Node &n) {
		id = n.id;
		g = n.g;
		sap = n.sap;
		heap_state = n.heap_state;
		is_inheap_node = n.is_inheap_node;
	}

	Node(int g, shared_ptr<StateActionPair> sap, StateID heap_state)
	:id(-1), g(g), sap(sap), heap_state(heap_state){
	};

	size_t hash() const{
		size_t value_sap = sap->hash();
		size_t value_heap =  this->heap_state.hash();
		size_t seed = 0;
		utils::hash_combine(seed, value_sap);
		utils::hash_combine(seed, value_heap);
		utils::hash_combine(seed, id);
		return seed;
	}

    bool operator==(const Node& other) const {
		if (id != other.id)
			return false;
		if (g != other.g)
			return false;
		if ((!sap && other.sap) || (sap && !other.sap))
            return false;

        if (*sap != *other.sap)
			return false;

		if (heap_state != other.heap_state)
			return false;
		return true;
	};

    bool operator<(const Node &other) const {
		if (other.g < g)
            return true;
        if (other.g > g)
			return false;
		// Tie Breaking if they have equal g_values
		size_t hash = this->hash();
		size_t other_hash = other.hash();
		if (other_hash  < hash)
			return true;
		return false;
	};
};

namespace std {
	template<>
	struct hash<StateActionPair> {
		std::size_t operator()(const StateActionPair &sap) const {
			return sap.hash();
		}
	};
    template<>
	struct hash<Node> {
		std::size_t operator()(const Node &node) const {
			return node.hash();
		}
	};
}

template <typename T> struct Cmp {
    bool operator() (const T &lhs, const T &rhs) {
		return *lhs < *rhs;
	};
};
#endif
