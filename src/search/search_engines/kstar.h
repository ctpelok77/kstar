#ifndef KSTAR_H
#define KSTAR_H

#include "kstar_util.h"
#include "top_k_eager_search.h"
#include "../algorithms/priority_queues.h"
#include <memory>

namespace kstar {
typedef shared_ptr<StateActionPair> Sap;
typedef std::pair<int, Sap> Node;

class KStar : public top_k_eager_search::TopKEagerSearch
{
private:
	int get_f_value(StateID id);

protected:		
	virtual ~KStar() = default;
	bool first_plan_found;
	int optimal_solution_cost;
	std::priority_queue<Node> queue_djkstra;
	std::unordered_set<StateID> heap_initialized;
    std::map<Sap, Sap> parent_sap;
	std::map<Sap, bool> cross_edge;
	std::unique_ptr<PlanReconstructor> plan_reconstructor;
	std::unordered_set<StateActionPair> closed;

	void djkstra_search();
	vector<Sap> djkstra_traceback(Node& top_pair);
	vector<Sap> compute_sidetrack_seq(Node& top_pair, vector<Sap>& path);
	bool is_initialized(GlobalState &s);
	void set_initialized(GlobalState &s);
	void init_plan_reconstructor();
	void reset_initialized();
	void add_plan(Node& p);
	void add_goal_heap_top();
	void add_first_plan();
	std::string get_node_name(Node &p);
	bool enough_plans_found();
	void notify_generate(Node &p);
	void notify_push(Node &p);
	void notify_expand(Node& p);
	void add_cross_edges(Node& p);
	int get_cost_heap_edge(Sap& from, Sap& to);
	int get_cost_cross_edge(Sap& to);
public:
	KStar (const options::Options &opts);
	void search() override;
};
}

#endif 
