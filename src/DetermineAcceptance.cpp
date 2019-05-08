//
// Created by wsc on 19-5-8.
//

#include "DetermineAcceptance.h"

bool DetermineAcceptance::transitionAccepted(IBestSolutionManager &bestSolutionManager, ISolution &currentSolution,
                                             ISolution &newSolution, ALNS_Iteration_Status &status) {
    return newSolution < currentSolution;
}
