#ifndef INCOMING_HEAP__H
#define INCOMING_HEAP__H

#include <vector>
#include <algorithm>

#include "../state_action_pair.h"
#include "../per_state_information.h"
#include "../utils/util.h"

namespace k_star_heaps {
template<class T> class IncomingHeap
{
		std::vector<T> bucket;
		bool sorted;
		size_t queue_top;
		
public:
		IncomingHeap() : sorted(false), queue_top(-1) { 
		};

		bool empty() {
			return bucket.size() == queue_top || bucket.empty();		
		}

		void push(T &element) {
			sorted = false;
			bucket.push_back(element);
			bucket.shrink_to_fit();
		}

		T& top() {
			if (sorted) {
				return bucket[queue_top];		
			}
			std::sort(bucket.begin(),  bucket.end());
			sorted = true;	
			queue_top = 0;
			return bucket[queue_top];
		}

		void pop() {
			++queue_top;
		}

		virtual ~IncomingHeap() = default;
};

typedef IncomingHeap<StateActionPair> InHeap;

class TreeHeap
{
		InHeap* ancestor_heap;
		StateActionPair root;
		bool root_popped;

public:
		TreeHeap ()
		:  root(StateActionPair::no_sap), root_popped(false){
		};
		
		void set_root(StateActionPair& root) {
			this->root = root;		
		}	

		void set_ancestor_heap(InHeap* ancestor_heap) {
			this->ancestor_heap = ancestor_heap; 		
		}

		bool empty() {
			if (!ancestor_heap) { 
				return true;
			}
			return ancestor_heap->empty();			
		}

		StateActionPair& top() {	
			if (!root_popped) 
				return root;	
			return ancestor_heap->top();
		};		

		void pop() {
			if (!root_popped) 
				root_popped = true;					
			ancestor_heap->pop();
		};

		virtual ~TreeHeap() = default;
};
}
#endif 
