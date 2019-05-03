#include <utility>

//
// Created by wsc on 19-5-2.
//

#ifndef ALNS_EFSMTW_MT_SEQUENTIALNODEINSERTION_H
#define ALNS_EFSMTW_MT_SEQUENTIALNODEINSERTION_H
#ifndef INF
#define INF 1000000
#endif
#include "ALNS_inc.h"
#include "logistics.hpp"

class SequentialNodeInsertion: public ARepairOperator {
public:
    explicit SequentialNodeInsertion(std::string s) : ARepairOperator(std::move(s)) {}

    ~SequentialNodeInsertion() override = default;

    void repairSolution(ISolution& sol) override ;
};


#endif //ALNS_EFSMTW_MT_SEQUENTIALNODEINSERTION_H
