#ifndef GRAPHVIZ_WRITER_H
#define GRAPHVIZ_WRITER_H

#include <sstream>

#include "per_state_information.h"
#include "algorithms/heaps.h"
#include "state_registry.h"
#include "search_space.h"
#include "state_action_pair.h"
#include "search_engines/kstar_util.h"

namespace graphviz_writer {
typedef PerStateInformation<k_star_heaps::IncomingHeap<shared_ptr<StateActionPair>>> Heap;
typedef shared_ptr<StateActionPair> Sap;
typedef std::stringstream Stream;
typedef kstar::SuccessorGenerator SuccGen;

class GraphvizWriter {
		StateRegistry* reg;		
		SearchSpace* ssp;
		void save_and_close(std::string filename, Stream &stream, Stream &node_stream);
	    void add_node(std::string id, std::string label, Stream &stream);
		void create_state_inheap(GlobalState s, Heap& heap,
								 Stream& stream,int &total_num_nodes);
		void add_nodes_inheap(GlobalState s, Heap& heap, Stream &stream, int &total_num_nodes);
		std::string get_node_name(StateActionPair &edge);
		std::string get_node_label(StateActionPair &edge);

public:
		GraphvizWriter(StateRegistry* reg, SearchSpace* ssp);
		// For dumping incomming heap H_in
		void dump_inheap(Heap& heap, std::string filename="inheap.dot");
		void dump_tree_heap(Heap& heap, , std::string filename="inheap.dot");
		virtual ~GraphvizWriter() = default;
};
}

#endif 
