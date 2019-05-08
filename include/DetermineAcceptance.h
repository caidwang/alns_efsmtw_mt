//
// Created by wsc on 19-5-8.
//

#ifndef ALNS_EFSMTW_MT_DITERMINEACCEPTANCE_H
#define ALNS_EFSMTW_MT_DETERMINEACCEPTANCE_H

#include "ALNS_inc.h"

class DetermineAcceptance : public IAcceptanceModule {
public:
    DetermineAcceptance() = default;
    ~DetermineAcceptance() = default;
    //! Indicate if the new created solution have to be accepted or not
    //! \param bestSolutionManager a reference to the best solution manager.
    //! \param currentSolution current solution.
    //! \param newSolution new solution.
    //! \param status the status of the current alns iteration.
    //! \return true if the transition is accepted, false otherwise.
    bool transitionAccepted(IBestSolutionManager& bestSolutionManager, ISolution& currentSolution, ISolution& newSolution, ALNS_Iteration_Status& status) override ;

    //! Some Acceptance modules needs to initialize some variable
    //! only when the solver actualy starts working. In this case
    //! you should override this method.
    void startSignal() override {};
};


#endif //ALNS_EFSMTW_MT_DITERMINEACCEPTANCE_H
