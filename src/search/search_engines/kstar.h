#ifndef KSTAR_H
#define KSTAR_H

#include "top_k_eager_search.h"
#include "../algorithms/priority_queues.h"

namespace kstar {
class KStar : public top_k_eager_search::TopKEagerSearch 
{
protected:		
	virtual ~KStar() = default;
	bool first_solution_found;
	priority_queues::AdaptiveQueue<top_k_eager_search::StateActionPair> queue_djkstra;
	void djkstra_search(); 
	void add_goal_sap();
public:
	KStar (const options::Options &opts);

	void search() override;

};
}

#endif 
