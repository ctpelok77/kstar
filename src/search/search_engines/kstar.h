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
	priority_queues::AdaptiveQueue<top_k_eager_search::StateActionPair> queue_djkstra;

	void djkstra_search(); 
	void add_goal_heap_top();
	void add_first_plan();
	void dump_astar_search_space();
	// TODO: implement this one
	void dump_djkstra_search();
public:
	KStar (const options::Options &opts);

	void search() override;

};
}

#endif 
