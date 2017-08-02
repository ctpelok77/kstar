#ifndef INCOMING_HEAP__H
#define INCOMING_HEAP__H

#include <vector>
#include <algorithm>

#include "../state_action_pair.h"
#include "../per_state_information.h"
#include "../utils/util.h"

namespace k_star_heaps {

template <typename T> struct Cmp {
	bool operator() (const T &lhs, const T &rhs) {
		return *lhs < *rhs;			
	};
};

template<class T> class IncomingHeap
{
public:
		std::vector<T> bucket;
		bool sorted;
		int queue_top;
		int forbidden = -1;

		
		IncomingHeap() : sorted(false), queue_top(0) , forbidden(-1) {
		};

		IncomingHeap(const IncomingHeap &other) {
			bucket = other.bucket;	
			sorted = other.sorted;
			queue_top = other.queue_top; 
		}

		IncomingHeap(const IncomingHeap &other, const T& new_top) {
			bucket = other.bucket;
			sorted = false;
            queue_top = other.queue_top;
			bucket.push_back(new_top);
		}

		bool empty() {
			// container empty
			if (bucket.empty()) 
				return true;
			int size = bucket.size();
			// one element which is forbidden
			if (queue_top == size - 1 && forbidden == queue_top)
				return true;
            // all elements iterated
			if (queue_top > size - 1)
                return true;
			return false;
		}

		void push(T& element) {
			sorted = false;
			bucket.push_back(element);
		}

		T& top() {
			if (queue_top == forbidden) {
				++queue_top; 		
			}	

			if (sorted) {
				return bucket[queue_top];		
			}
			
			std::sort(bucket.begin(), bucket.end(), Cmp<T>());
			sorted = true;
			return bucket[queue_top];
		}

		void pop() {
			++queue_top;
		}

		void forbid_top() {
			forbidden = queue_top;			
		}

		void reset() {
			queue_top = 0;			
		}

		virtual ~IncomingHeap() = default;
};

}
#endif 
