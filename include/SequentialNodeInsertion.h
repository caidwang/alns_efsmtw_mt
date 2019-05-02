#include <utility>

//
// Created by wsc on 19-5-2.
//

#ifndef ALNS_EFSMTW_MT_SEQUENTIALNODEINSERTION_H
#define ALNS_EFSMTW_MT_SEQUENTIALNODEINSERTION_H

#include "ALNS_inc.h"

class SequentialNodeInsertion: public ARepairOperator {
public:
    SequentialNodeInsertion(std::string s) : ARepairOperator(std::move(s)) {}

    virtual ~SequentialNodeInsertion(){};

    void repairSolution(ISolution& sol) override ;
};


#endif //ALNS_EFSMTW_MT_SEQUENTIALNODEINSERTION_H
