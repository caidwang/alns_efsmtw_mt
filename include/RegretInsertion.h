//
// Created by wsc on 19-5-2.
//

#ifndef ALNS_EFSMTW_MT_REGRETINSERTION_H
#define ALNS_EFSMTW_MT_REGRETINSERTION_H

#include "ALNS_inc.h"

class RegretInsertion: public ARepairOperator {
public:
    RegretInsertion(std::string s, int k) : ARepairOperator(std::move(s)), k(k) {}
    ~RegretInsertion() override = default;
    void repairSolution(ISolution& sol) override ;

private:
    int k;
};

#endif //ALNS_EFSMTW_MT_REGRETINSERTION_H
