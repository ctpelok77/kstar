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

		IncomingHeap(const IncomingHeap &other) {
			bucket = std::vector<T>(other.bucket);	
			sorted = other.sorted;
			queue_top = 0; 
		}

		bool empty() {
			return bucket.size() == queue_top || bucket.empty();		
		}

		size_t size() {
			return bucket.size();	
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

		void reset() {
			queue_top = 0;			
		}

		virtual ~IncomingHeap() = default;
};

typedef IncomingHeap<StateActionPair> InHeap;
}
#endif 
