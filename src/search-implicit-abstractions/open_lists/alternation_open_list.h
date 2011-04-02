#ifndef OPEN_LISTS_ALTERNATION_OPEN_LIST_H
#define OPEN_LISTS_ALTERNATION_OPEN_LIST_H

#include "open_list.h"
#include "../evaluator.h"

#include <vector>

template<class Entry>
class AlternationOpenList : public OpenList<Entry> {
    std::vector<OpenList<Entry> *> open_lists; 
    std::vector<int> priorities;

    int size;
    bool dead_end;
    bool dead_end_reliable;
    int boosting;
    int last_used_list;

protected:
    Evaluator* get_evaluator() { return this; }

public:
    AlternationOpenList(const vector<OpenList<Entry> *> &sublists,
            int boost_influence=1000); 
    // roughly speaking, boost_influence is how often the boosted queue should be
    // preferred when removing an entry

    ~AlternationOpenList();

    // OpenList interface
    int insert(const Entry &entry);
    Entry remove_min();
    bool empty() const;
    void clear();

    // Evaluator interface
    void evaluate(int g, bool preferred);
    bool is_dead_end() const;
    bool dead_end_is_reliable() const;
    
    int boost_preferred();
    void boost_last_used_list();
    virtual const std::vector<int> &get_last_key_removed() const;
};

#include "alternation_open_list.cc"

// HACK! Need a better strategy of dealing with templates, also in the Makefile.

#endif
