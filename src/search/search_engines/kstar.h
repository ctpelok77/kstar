#ifndef KSTAR_H
#define KSTAR_H

#include "top_k_eager_search.h"
#include "../algorithms/priority_queues.h"
#include <memory>

namespace kstar {
	typedef shared_ptr<StateActionPair> s_StateActionPair;
class KStar : public top_k_eager_search::TopKEagerSearch
{
private:
	int get_f_value(StateID id);

protected:		
	virtual ~KStar() = default;
	bool first_plan_found;
	int optimal_solution_cost;
	std::priority_queue<std::pair<int, s_StateActionPair>> queue_djkstra;
	PerStateInformation<bool> heap_initialized;
    std::map<s_StateActionPair, s_StateActionPair> parent_sap;
	std::map<s_StateActionPair, bool> cross_edge;

	void djkstra_search();

	vector<s_StateActionPair> djkstra_traceback(std::pair<int, s_StateActionPair>& top_pair);
	vector<s_StateActionPair> compute_sidetrack_seq(std::pair<int, s_StateActionPair>& top_pair,
													vector<s_StateActionPair>& path);
	void add_plan(std::pair<int, s_StateActionPair>& p);
	void add_goal_heap_top();
	void add_cross_edges(std::pair<int, s_StateActionPair> p);
	void add_first_plan();
	void dump_astar_search_space();
	std::string get_node_name(std::pair<int, s_StateActionPair>& p);
	void notify_generate(std::pair<int, s_StateActionPair>& p);
	void notify_push(std::pair<int, s_StateActionPair>& p);
	void notify_expand(std::pair<int, s_StateActionPair>& p);
	int get_cost_heap_edge(s_StateActionPair& from, s_StateActionPair& to);
	int get_cost_cross_edge(s_StateActionPair& to);
public:
	KStar (const options::Options &opts);
	void search() override;

};
}

#endif 
