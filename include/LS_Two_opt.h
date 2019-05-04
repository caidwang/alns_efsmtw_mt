//
// Created by wsc on 19-5-2.
//

#ifndef ALNS_EFSMTW_MT_LS_TWO_OPT_H
#define ALNS_EFSMTW_MT_LS_TWO_OPT_H

#include "ALNS_inc.h"
#include "logistics.hpp"
#include "VRP_Solution.h"
class LS_Two_opt : public ILocalSearch {
public:
    explicit LS_Two_opt(std::string name) : name(std::move(name)) {}
    //! Perform a local search on the solution.
    //! \return true if the solution is improved.
    bool performLocalSearch(ISolution& sol) override ;

    //! \return the name of the local search operator.
    std::string getName() override {return name; };

private:
    std::string name;
};


#endif //ALNS_EFSMTW_MT_LS_TWO_OPT_H
