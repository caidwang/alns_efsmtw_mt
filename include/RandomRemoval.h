//
// Created by wsc on 19-5-1.
//

#ifndef ALNS_EFSMTW_MT_RANDOMREMOVAL_H
#define ALNS_EFSMTW_MT_RANDOMREMOVAL_H

#ifndef MIN_Q_RATE
#define MIN_Q_RATE 0.1
#define MAX_Q_RATE 0.15
#endif

#include "ALNS_inc.h"

class RandomRemoval: public ADestroyOperator {
public:
    RandomRemoval(std::string name) : ADestroyOperator(10, 15, std::move(name)) {}
    ~RandomRemoval() override = default;
    void destroySolution(ISolution& sol) override;
};
#endif //ALNS_EFSMTW_MT_RANDOMREMOVAL_H
