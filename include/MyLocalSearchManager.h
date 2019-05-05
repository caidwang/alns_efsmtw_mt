//
// Created by wsc on 19-5-5.
//

#ifndef ALNS_EFSMTW_MT_MYLOCALSEARCHMANAGER_H
#define ALNS_EFSMTW_MT_MYLOCALSEARCHMANAGER_H

#include <vector>
#include <algorithm>
#include <random>
#include "ALNS_inc.h"

class MyLocalSearchManager: public ILocalSearchManager {
public:
    explicit MyLocalSearchManager(ALNS_Parameters& parameters) {param = &parameters;}
    virtual ~MyLocalSearchManager() = default;

    //! \param sol the solution to be improved.
    //! \param status the status of the alns iteration.
    //! \return true if the solution has been improved.
    bool useLocalSearch(ISolution& sol, ALNS_Iteration_Status& status) override ;

    //! Add a local search operator to the manager.
    void addLocalSearchOperator(ILocalSearch& ls);


    void startSignal() override {};
private:
    //! A vector containing the local search operators managed by the current instance.
    std::vector<ILocalSearch*> localSearchOperators;

    //! Parameters of the ALNS.
    ALNS_Parameters* param;
};


#endif //ALNS_EFSMTW_MT_MYLOCALSEARCHMANAGER_H
