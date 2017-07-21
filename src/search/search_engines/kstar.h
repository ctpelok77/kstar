#ifndef KSTAR_H
#define KSTAR_H

#include "top_k_eager_search.h"
#include "../algorithms/priority_queues.h"

namespace kstar {
class KStar : public top_k_eager_search::TopKEagerSearch 
{
private:
	int get_f_value(StateID id);

protected:		
	virtual ~KStar() = default;
	bool first_plan_found;
	int optimal_solution_cost;
	std::priority_queue<std::pair<int, StateActionPair>> queue_djkstra;
	void djkstra_search(); 
	void add_goal_heap_top();
	void add_first_plan();
	void dump_astar_search_space();
	std::string get_node_name(StateActionPair& p);
	void notify_generate(StateActionPair& p);
	void notify_push(StateActionPair& p);
	void notify_expand(StateActionPair& p);
	int get_cost_heap_edge(StateActionPair& from, StateActionPair& to);
	int get_cost_cross_edge(StateActionPair& to);
public:
	KStar (const options::Options &opts);
	void search() override;

};
}

#endif 
