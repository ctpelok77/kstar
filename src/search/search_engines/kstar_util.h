#ifndef KSTAR_UTIL
#define KSTAR_UTIL 

#include <vector>
#include <memory>
#include "../state_action_pair.h"
#include "../global_operator.h"
#include "../algorithms/heaps.h"

namespace kstar {
typedef shared_ptr<StateActionPair> Sap;
typedef std::pair<int, Sap> Node;
typedef std::vector<const GlobalOperator*> Plan;
typedef k_star_heaps::IncomingHeap<Sap> InHeap;

class PlanReconstructor {
	std::map<Sap, Sap> &parent_sap;
	std::map<Sap, bool> &cross_edge;
	StateID goal_state;	
	StateRegistry* state_registry;
	SearchSpace* search_space;

public:
	PlanReconstructor(std::map<Sap, Sap>& parent_sap, 
					   std::map<Sap, bool>& cross_edge,
					   StateID goal_state,
					   StateRegistry* state_registry,
					   SearchSpace* search_space);
	
	virtual ~PlanReconstructor() = default;
	std::vector<Sap> djkstra_traceback(Node& top_pair);
	std::vector<Sap> compute_sidetrack_seq(std::vector<Sap>& path);
	void add_plan(Node& top_pair, std::vector<Plan>& top_k_plans);
};

class SuccessorGenerator {
	PerStateInformation<InHeap> &H_T;
	std::map<Sap, Sap> &parent_sap;
	std::map<Sap, bool> &cross_edge;
	StateRegistry* state_registry;
		
public:
	SuccessorGenerator(PerStateInformation<InHeap> &H_T, 
					   std::map<Sap, Sap> &parent_sap,
					   std::map<Sap, bool> &cross_edge, 
					   StateRegistry* state_registry);

	virtual ~SuccessorGenerator() = default;
	void get_successor_R(Node &sap, Node &successor);
	void get_successors(Node &sap, vector<Node> &successor_sap);
	int get_max_successor_delta(Node& sap);
	int get_cost_cross_edge(Sap& to);
	int get_cost_heap_edge(Sap& from, Sap& to);
    void add_cross_edge(Node &p, vector<Node> &successors);
};
}
#endif 
