#include "task.h"

using namespace std;

OperatorRef::OperatorRef(const TaskImpl &impl_, size_t index_)
    : impl(impl_),
      index(index_) {}

OperatorRef::~OperatorRef() {}

OperatorsRef::OperatorsRef(const TaskImpl &impl_)
    : impl(impl_) {}

OperatorsRef::~OperatorsRef() {}

Task::Task(const TaskImpl &impl_)
    : impl(impl_) {}

Task::~Task() {}
