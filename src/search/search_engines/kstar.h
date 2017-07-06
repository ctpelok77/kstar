#ifndef KSTAR_H
#define KSTAR_H

#include "top_k_eager_search.h"

namespace kstar {
class KStar : public top_k_eager_search::TopKEagerSearch 
{
protected:		
	virtual ~KStar() = default;
public:
	KStar (const options::Options &opts);
	void search() override;

};
}

#endif 
