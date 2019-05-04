//
// Created by wsc on 19-5-4.
//

#ifndef ALNS_EFSMTW_MT_LS_RELOCATE_H
#define ALNS_EFSMTW_MT_LS_RELOCATE_H

#include "logistics.hpp"
#include "ALNS_inc.h"

class LS_Relocate: public ILocalSearch  {
public:
    explicit LS_Relocate(std::string name) : name(name) { }
    //! Perform a local search on the solution.
    //! \return true if the solution is improved.
    bool performLocalSearch(ISolution& sol) override ;

    //! \return the name of the local search operator.
    std::string getName() override {return name; }
private:
    std::string name;
};


#endif //ALNS_EFSMTW_MT_LS_RELOCATE_H
