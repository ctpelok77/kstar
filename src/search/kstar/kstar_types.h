#include <memory>
#include "../state_action_pair.h"

using namespace std;

namespace kstar {
	typedef shared_ptr<StateActionPair> Sap;
	typedef std::pair<Node, Node> Edge;
	typedef std::vector<const GlobalOperator*> Plan;
	typedef std::vector<StateID> StateSequence;
}
