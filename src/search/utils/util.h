#ifndef UTIL
#define UTIL

#include <map>
#include <set>
#include <sstream>
#include "../search_node_info.h"
#include "../global_operator.h"
#include "../globals.h"

using std::string;
using std::cout;
using std::flush;
using std::endl;
using std::map;
using std::cout;
using std::endl;
using std::flush;
using std::vector;
using std::ofstream;
using Info = const SearchNodeInfo; 

using namespace std;

inline void print_set_of_operators(vector<GlobalOperator>& ops)
{
	int index = 0;

	for (auto& o: ops) {
		std::cout << "Operator" << index  << std::endl;
		o.dump();	
		++index;
	}	

}


inline void print_set_of_operators(vector<const GlobalOperator*>& ops)
{

	for (auto& o: ops) {
		o->dump();	
	}	

}

inline void print_goal()
{

	for (size_t i = 0; i < g_goal.size(); ++i) {
		std::cout << "var"<< g_goal[i].first << "=" << g_goal[i].second << std::endl;		
	}	
	
}

inline void update_maximum(int& current_value, int& old_value) {
	if (current_value > old_value) 
		old_value = current_value;	
}

inline std::string state_label(GlobalState& s)
{
	std::string state_string ;
	for (size_t i = 0; i < g_variable_domain.size(); ++i) {
		state_string += std::to_string(s[i]);		
	}
	return state_string;	
}

inline std::string state_label(const GlobalState& s)
{
	std::string state_string ;
	for (size_t i = 0; i < g_variable_domain.size(); ++i) {
		state_string += "var"+std::to_string(i)+ "="+ std::to_string(s[i]) + "\n";		
	}
	return state_string;	
}

inline void print_state(const GlobalState& s)
{
	std::cout << state_label(s);; 
}

template<class T>
bool is_subset(std::set<T>&v1, std::set<T>&v2) {
	for (T element: v1) {
		if (v2.find(element) == v2.end()) return false;
	}

	return true;
}

template<class T>
void print_set(std::set<T> set)
{
	for (auto& e : set) {
		std::cout << e << std::endl;	
	}	
}

inline void add_node(std::string id, std::string label, std::stringstream &stream)
{
	stream << id << "[peripheries=\"1\", shape=\"rectangle\", style=\"rounded, filled\"";  
	stream << "fillcolor=\"yellow\", label=\" "<< label << "\" ]" << endl;
}


inline void add_edge(std::string from_id, std::string to_id, std::string label, std::stringstream &stream)
{
	stream << from_id <<  " -> " << to_id << "[label=\"" << label << "\"]" << endl;
}
#endif 
