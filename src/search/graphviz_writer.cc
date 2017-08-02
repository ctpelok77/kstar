#include "graphviz_writer.h"
#include "utils/util.h"

namespace graphviz_writer {

GraphvizWriter::GraphvizWriter(StateRegistry* reg, SearchSpace* ssp) 
:reg(reg), ssp(ssp){
		
}

void GraphvizWriter::create_state_inheap(GlobalState s, Heap &heap, 
										 Stream &stream, int &total_num_nodes)
{
	stream << "subgraph " << "cluster_" << s.get_id().get_value()  << " {" << endl; 	
	stream << "label=" << "node" <<  s.get_id().get_value() << ";" <<  endl;
	stream << "style=filled;" << endl;
	stream << "color=grey;"  << endl;
	add_nodes_inheap(s, heap, stream, total_num_nodes);
	stream << "}" << endl;
}

std::string GraphvizWriter::get_node_label(StateActionPair &edge) {
    int from = edge.from.get_value();
	int to = edge.to.get_value();
	std::string node_name = "("+ std::to_string(from) + ", " + std::to_string(to) + ")"
							+ " delta: " + std::to_string(edge.get_delta()); 
	return node_name;
}


std::string GraphvizWriter::get_node_name(StateActionPair &edge) {
	int from = edge.from.get_value();
	int to = edge.to.get_value();
	std::string node_name = std::to_string(from) + std::to_string(to);
	return node_name;
}

void GraphvizWriter::add_node(std::string id, std::string label, Stream &stream)
{
	stream << id << "[peripheries=\"1\", shape=\"rectangle\", style=\"rounded, filled\"";  
	stream << "fillcolor=\"yellow\", label=\" "<< label << "\" ]" << endl;
}

void GraphvizWriter::add_nodes_inheap(GlobalState s, Heap& heap, 
									  Stream &stream, int &total_num_nodes) {

	Sap predecessor = nullptr;
	while (!heap[s].empty()) {
		Sap sap = heap[s].top();
		string node_label = get_node_label(*sap);
		string node_name = get_node_name(*sap);
		
		add_node(node_name, node_label, stream);
		heap[s].pop();
		if (predecessor) {
			std::string pred_name = get_node_name(*predecessor);
			add_edge(pred_name, node_name, "" , stream);
		}

		predecessor = sap; 
		++total_num_nodes;
	}	
	
	heap[s].reset();
}

void GraphvizWriter::save_and_close(std::string filename,
									Stream &stream,
									Stream &node_stream) {
	stream << "}" << endl; 	
	std::ofstream file;
	file.open(filename, std::ofstream::out);
	file << stream.rdbuf();
	file << node_stream.rdbuf();
	file.close();	
}

void GraphvizWriter::dump_inheap(Heap &heap, std::string filename) {
	std::stringstream stream, node_stream;	
	int total_num_nodes = 0;

	stream << "digraph {\n" << endl ;
	PerStateInformation<SearchNodeInfo>& search_node_infos = ssp->search_node_infos;
	for (PerStateInformation<SearchNodeInfo>::const_iterator it =\
		search_node_infos.begin(reg); 
		it != search_node_infos.end(reg); ++it) {
		StateID id = *it;
		GlobalState s = reg->lookup_state(id); 
		create_state_inheap(s, heap, stream, total_num_nodes);
	}
	save_and_close(filename, stream, node_stream);
}

void GraphvizWriter::dump_tree_heap(Heap &heap, std::string filename) {
	std::stringstream stream, node_stream;	
	int total_num_nodes = 0;

	stream << "digraph {\n" << endl ;
	PerStateInformation<SearchNodeInfo>& search_node_infos = ssp->search_node_infos;
	for (PerStateInformation<SearchNodeInfo>::const_iterator it =\
		search_node_infos.begin(reg); 
		it != search_node_infos.end(reg); ++it) {
		StateID id = *it;
		GlobalState s = reg->lookup_state(id); 
		create_state_inheap(s, heap, stream, total_num_nodes);
	}
	save_and_close(filename, stream, node_stream);
}

}
