#ifndef KSTAR_H
#define KSTAR_H

#include "kstar_util.h"
#include "top_k_eager_search.h"
#include "../algorithms/priority_queues.h"
#include "../graphviz_writer.h"

#include <memory>

namespace kstar {
typedef shared_ptr<StateActionPair> Sap;
typedef std::pair<int, Sap> Node;

class KStar : public top_k_eager_search::TopKEagerSearch
{
private:
	int get_f_value(StateID id);
	void initialize_tree_heaps(Sap& sap);

protected:		
	virtual ~KStar() = default;
	int optimal_solution_cost;
	std::priority_queue<Node> queue_djkstra;
	std::unordered_set<StateID> heap_initialized;
    std::map<Sap, Sap> parent_sap;
	std::map<Sap, bool> cross_edge;
	std::unique_ptr<PlanReconstructor> plan_reconstructor;
	std::unique_ptr<SuccessorGenerator> pg_succ_generator;
	std::unique_ptr<graphviz_writer::GraphvizWriter> graphviz_writer;
	std::unordered_set<StateActionPair> closed;

	// djkstra search return true if k solutions have been found and false otherwise
	bool djkstra_search();
	bool enough_nodes_expanded();
	void resume_astar();
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
public:
	KStar (const options::Options &opts);
	void search() override;
};
}

#endif 
