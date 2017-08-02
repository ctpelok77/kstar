#ifndef STATE_ACTION_PAIR
#define STATE_ACTION_PAIR 

#include "state_id.h"
#include "state_registry.h"
#include "search_space.h"
#include "global_operator.h"
#include "utils/util.h"


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
		const GlobalState to_state = reg->lookup_state(to);
		const GlobalState from_state = reg->lookup_state(from);
		SearchNode from_node = ssp->get_node(from_state);
		SearchNode to_node = ssp->get_node(to_state);

		return from_node.get_g() + op->get_cost() - to_node.get_g();
	};


	bool operator<(const StateActionPair &other) const {            
		return this->get_delta() <other.get_delta();
	};

	bool operator==(const StateActionPair& other) const {
		if (from == other.from || to == other.to)
			return true;
		return false;
	};

	bool operator!=(const StateActionPair &other) const {
		if (*this == other) 
			return true;		
		return false;
	}

	// TODO:
	void dump() {
		/*std::cout << "from "<< from << flush << std::endl;
		std::cout << "to "<< to << flush << std::endl;
		if (op) {
			std::cout << op->get_name() << flush << std::endl;
		}
		std::cout << "" << std::endl;
		*/
		std::cout << "(" << from <<","<< to << ")" << endl;
	}
};

namespace std {
	template<>
	struct hash<StateActionPair> {
		std::size_t operator()(const StateActionPair &sap) const {
            std::hash<long int> hash_function;
            long int value_from = sap.from.get_value();
			long int value_to = sap.to.get_value();
			return hash_function(value_from) + hash_function(value_to);
		}
	};
}
#endif 
