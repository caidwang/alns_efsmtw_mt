//
// Created by wsc on 19-5-1.
//

#ifndef ALNS_EFSMTW_MT_RANDOMANDRELATEDREMOVAL_H
#define ALNS_EFSMTW_MT_RANDOMANDRELATEDREMOVAL_H

#include "ALNS_inc.h"
#include "Relatedness.h"
#include "logistics.hpp"

class RandomAndRelatedRemoval : public ADestroyOperator{
public:
    RandomAndRelatedRemoval(std::string name, Relatedness &relatedness) : ADestroyOperator(10, 15, std::move(name)), relatedness(relatedness) {}
    ~RandomAndRelatedRemoval() = default;
    void destroySolution(ISolution& sol) override ;
private:
    const Relatedness &relatedness;
};


#endif //ALNS_EFSMTW_MT_RANDOMANDRELATEDREMOVAL_H
