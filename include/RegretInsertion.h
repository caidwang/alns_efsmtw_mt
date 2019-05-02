//
// Created by wsc on 19-5-2.
//

#ifndef ALNS_EFSMTW_MT_REGRETINSERTION_H
#define ALNS_EFSMTW_MT_REGRETINSERTION_H

#include "ALNS_inc.h"

class RegretInsertion: public ARepairOperator {
public:
    RegretInsertion(std::string s) : ARepairOperator(std::move(s)) {}
    ~RegretInsertion() = default;
    void repairSolution(ISolution& sol) override ;
};

#endif //ALNS_EFSMTW_MT_REGRETINSERTION_H
