/* ALNS_Framework - a framework to develop ALNS based solvers
 *
 * Copyright (C) 2012 Renaud Masson
 *
 * This library is free software; you can redistribute it and/or
 * modify it either under the terms of the GNU Lesser General Public
 * License version 3 as published by the Free Software Foundation
 * (the "LGPL"). If you do not alter this notice, a recipient may use
 * your version of this file under the LGPL.
 *
 * You should have received a copy of the LGPL along with this library
 * in the file COPYING-LGPL-3; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Suite 500, Boston, MA 02110-1335, USA
 *
 * This software is distributed on an "AS IS" basis, WITHOUT WARRANTY
 * OF ANY KIND, either express or implied. See the LGPL for
 * the specific language governing rights and limitations.
 *
 * The Original Code is the ALNS_Framework library.
 *
 *
 * Contributor(s):
 *	Renaud Masson
 *
 *	ALNS adapted for EFSMTW_MT
 *
 *	Change the procedure of Func performOneIteration. Add the process of generating
 *	Feasible solution from the current best solution.
 *
 *	Contributor:
 *	Caid Wang
 */
#include <assert.h>
#include <float.h>
#include <iostream>
#include <stdlib.h>
#include <set>
#include <time.h>
#include "ALNS.h"
#include "ALNS_Parameters.h"
#include "ALNS_Iteration_Status.h"
#include "ISolution.h"
#include "AOperatorManager.h"
#include "ARepairOperator.h"
#include "ADestroyOperator.h"
#include "AOperator.h"
#include "IUpdatable.h"
#include "IBestSolutionManager.h"
#include "../localsearch/ILocalSearchManager.h"
#include "../acceptanceModule/IAcceptanceModule.h"
#include "../statistics/Statistics.h"

using namespace std;


ALNS::ALNS(string instanceName,
		 ISolution& initialSolution,
		 IAcceptanceModule& acceptanceCrit,
		 ALNS_Parameters& parameters,
		 AOperatorManager& opMan,
		 IBestSolutionManager& bestSolMan,
		 IBestSolutionManager& bestFeasibleSolMan,
		 ILocalSearchManager& lsMan)
{
	name = instanceName;
	currentSolution = initialSolution.getCopy();
	acceptanceCriterion = &acceptanceCrit;
	param = &parameters;
	lowerBound = -DBL_MAX;
	nbIterationsWC = 0;
	nbIterations = 0;
	nbIterationsWithoutImprovement = 0;
	opManager = &opMan;
	bestSolManager = &bestSolMan;
	bestFeasibleSolManager = &bestFeasibleSolMan;
	lsManager = &lsMan;

	opManager->setStatistics(&stats);

	// We add the initial solution in the best solution manager.
	bestSolManager->isNewBestSolution(initialSolution);

	nbIterationsWithoutImprovementCurrent = 0;

	nbIterationsWithoutTransition = 0;

	nbIterationsWithoutLocalSearch = 0;


}

ALNS::~ALNS()
{
	delete currentSolution;
}


bool ALNS::solve()
{
	startingTime = clock();
	param->setLock();
	acceptanceCriterion->startSignal();
	opManager->startSignal();
	stats.setStart();
	while(!isStoppingCriterionMet())
	{
		performOneIteration();
	}
	string pathGlob = param->getStatsGlobPath();
	pathGlob += name;
	pathGlob += ".txt";
	string pathOp = param->getStatsOpPath();
	pathOp += name;
	pathOp += ".txt";
	stats.generateStatsFile(pathGlob,pathOp);
	return (*(bestSolManager->begin()))->isFeasible();
}

void ALNS::performOneIteration()
{

	status.partialReinit();

	ARepairOperator& repair = opManager->selectRepairOperator();
	ADestroyOperator& destroy = opManager->selectDestroyOperator();

	ISolution* newSolution = currentSolution->getCopy();

	if(nbIterations % param->getLogFrequency() == 0)
	{
		cout << "[ALNS] it. " << nbIterations << " best sol: " << (*(bestSolManager->begin()))->getPenalizedObjectiveValue() << " best Fsol: " << (*(bestFeasibleSolManager->begin()))->getObjectiveValue() << " nb known solutions: " << knownKeys.size() << endl;
	}

	destroy.destroySolution(*newSolution);

	status.setAlreadyDestroyed(ALNS_Iteration_Status::TRUE);
	status.setAlreadyRepaired(ALNS_Iteration_Status::FALSE);
	for(vector<IUpdatable*>::iterator it = updatableStructures.begin(); it != updatableStructures.end(); it++) {
        (*it)->update(*newSolution, status);
    }
	// 这里进行了update
	repair.repairSolution(*newSolution);
	status.setAlreadyRepaired(ALNS_Iteration_Status::TRUE);

	nbIterations++;
	status.setIterationId(nbIterations);
	nbIterationsWC++;

	double newCost = newSolution->getObjectiveValue();
	bool newBest = isNewBest(newSolution); // 如果更优 这里就已经把新解存起来了
	checkAgainstKnownSolution(*newSolution);
	bool betterThanCurrent = (*newSolution) < (*currentSolution);
	// 更新和noImprovement相关的信息
	if(betterThanCurrent)
	{
		nbIterationsWithoutImprovementCurrent = 0;
		status.setImproveCurrentSolution(ALNS_Iteration_Status::TRUE);
	}
	else
	{
		nbIterationsWithoutImprovementCurrent++;
		status.setImproveCurrentSolution(ALNS_Iteration_Status::FALSE);
	}
	status.setNbIterationWithoutImprovementCurrent(nbIterationsWithoutImprovementCurrent);

    // Local Search部分
	if(param->getPerformLocalSearch() && lsManager->useLocalSearch(*newSolution,status))
	{

	    newBest = bestSolManager->isNewBestSolution(*newSolution);
	}

	// 判断是否接受当前解 如果是SA策略 当当前解不是最优时也可能接受 但是对于DA来说只有更优才接受
	bool transitionAccepted = transitionCurrentSolution(newSolution);

	if(transitionAccepted)
	{
		status.setAcceptedAsCurrentSolution(ALNS_Iteration_Status::TRUE);
		nbIterationsWithoutTransition = 0;
	}
	else
	{
		status.setAcceptedAsCurrentSolution(ALNS_Iteration_Status::FALSE);
		nbIterationsWithoutTransition++;
	}
	status.setNbIterationWithoutTransition(nbIterationsWithoutTransition);

    // 在原本的ls后增加make feasible的步骤, 对最优的可行解 单独管理
    // 通过成倍的提高penalty 逼迫local search 向可行解方向搜索
    if (newBest) {
        int multiple = 1;
        for (int i = 0; (!newSolution->isFeasible()) && i < 2; ++i) {
            multiple *= 100;
            newSolution->setPenaltyMultiple(multiple);
            if(param->getPerformLocalSearch() )
                lsManager->useLocalSearch(*newSolution,status);
        }
    }
    newSolution->setPenaltyMultiple(1);
    if (newSolution->isFeasible())
        bestFeasibleSolManager->isNewBestSolution(*newSolution);



	opManager->updateScores(destroy,repair,status);


	stats.addEntry(static_cast<double>(clock()-startingTime)/CLOCKS_PER_SEC,nbIterations,destroy.getName(),repair.getName(),newCost,currentSolution->getObjectiveValue(),(*(bestSolManager->begin()))->getObjectiveValue(),knownKeys.size());


	if(nbIterationsWC % param->getTimeSegmentsIt() == 0)
	{
		opManager->recomputeWeights();
		nbIterationsWC = 0;
	}

	if (nbIterations % param->getSaveFrequency() == 0) {
        // todo 应该是feasibleBestManager写最优解
	    bestSolManager->saveBestAnswer();
	    bestFeasibleSolManager->saveBestAnswer();
	}

	for(vector<IUpdatable*>::iterator it = updatableStructures.begin(); it != updatableStructures.end(); it++)
	{
		(*it)->update(*newSolution,status);
	}

	currentSolution = bestSolManager->reloadBestSolution(currentSolution,status);

	delete newSolution;
}

bool ALNS::checkAgainstKnownSolution(ISolution& sol)
{
	bool notKnownSolution = false;
	long long keySol = sol.getHash();

	if(knownKeys.find(keySol) == knownKeys.end())
	{
		notKnownSolution = true;
		knownKeys.insert(keySol);
	}

	if(!notKnownSolution)
	{
		status.setAlreadyKnownSolution(ALNS_Iteration_Status::TRUE);
	}
	else
	{
		status.setAlreadyKnownSolution(ALNS_Iteration_Status::FALSE);
	}
	return notKnownSolution;
}

bool ALNS::isNewBest(ISolution* newSol)
{
	if(bestSolManager->isNewBestSolution(*newSol))
	{
		status.setNewBestSolution(ALNS_Iteration_Status::TRUE);
		nbIterationsWithoutImprovement = 0;
		status.setNbIterationWithoutImprovement(nbIterationsWithoutImprovement);
		status.setNbIterationWithoutImprovementSinceLastReload(0);
		return true;
	}
	else
	{
		status.setNewBestSolution(ALNS_Iteration_Status::FALSE);
		nbIterationsWithoutImprovement++;
		status.setNbIterationWithoutImprovement(nbIterationsWithoutImprovement);
		status.setNbIterationWithoutImprovementSinceLastReload(status.getNbIterationWithoutImprovementSinceLastReload()+1);
		return false;
	}
}

bool ALNS::transitionCurrentSolution(ISolution* newSol)
{

	if(acceptanceCriterion->transitionAccepted(*bestSolManager,*currentSolution,*newSol,status))
	{
		delete currentSolution;
		currentSolution = newSol->getCopy();
		return true;
	}
	else
	{
		return false;
	}
}

bool ALNS::isStoppingCriterionMet()
{
	if((*(bestSolManager->begin()))->isFeasible() && (*(bestSolManager->begin()))->getObjectiveValue() == lowerBound)
	{
		return true;
	}
	else
	{
		switch(param->getStopCrit())
		{
			case ALNS_Parameters::MAX_IT: {
				return nbIterations >= param->getMaxNbIterations();
			}
			case ALNS_Parameters::MAX_RT: {
				clock_t currentTime = clock();
				double elapsed = (static_cast<double>(currentTime - startingTime)) / CLOCKS_PER_SEC;
				return elapsed >= param->getMaxRunningTime();
			}
			case ALNS_Parameters::MAX_IT_NO_IMP: {
				return nbIterationsWithoutImprovement >= param->getMaxNbIterationsNoImp();
			}
			case ALNS_Parameters::ALL: {
				if(nbIterations >= param->getMaxNbIterations())
				{
					return true;
				}
				if(nbIterationsWithoutImprovement >= param->getMaxNbIterationsNoImp())
				{
					return true;
				}
				clock_t currentTime = clock();
				double elapsed = (static_cast<double>(currentTime - startingTime)) / CLOCKS_PER_SEC;
				if(elapsed >= param->getMaxRunningTime())
				{
					return true;
				}
				return false;
			}

			default: {
				assert(false);
				return false;
			}
		}
	}

}

void ALNS::end()
{
	opManager->end();
	delete opManager;
	delete acceptanceCriterion;
	delete lsManager;
	delete bestSolManager;
	delete bestFeasibleSolManager;
}
