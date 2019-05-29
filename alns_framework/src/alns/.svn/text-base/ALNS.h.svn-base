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
 */


#ifndef ALNS_H_
#define ALNS_H_

#include <cstring>
#include <math.h>
#include <time.h>
#include <set>
#include <vector>
#include "../statistics/Statistics.h"
#include "ALNS_Iteration_Status.h"

class IAcceptanceModule;
class ISolution;
class IUpdatable;
class ALNS_Parameters;
class AOperatorManager;
class ARepairOperator;
class ADestroyOperator;
class Statistics;
class IBestSolutionManager;
class ILocalSearchManager;

/*!
 * \class ALNS.
 * \brief This class contains the ALNS logic.
 *
 *
 * This class contains the logic of the ALNS (Adaptive Large Neighborhood Search).
 * The general idea of the ALNS is to iteratively destroy then repair a solution
 * to improve its quality. Non improving solution may be accepted as the new current
 * solution according to some acceptance criteria.
 * If you are interested about the general functionning of this method please refer to:
 * S. Ropke & D. Pisinger. An Adaptive Large Neighborhood Search Heuristic for Pickup
 * and Delivery Problem with Time Windows. Transportation Science, 40 (2006) 455-472.
 */

class ALNS {
private:

	//! The current solution.
	ISolution* currentSolution;

	//! The acceptance criteria to be used.
	IAcceptanceModule* acceptanceCriterion;

	//! The parameters to be used.
	ALNS_Parameters* param;

	//! Manager of the operators.
	AOperatorManager* opManager;

	//! Manager of the best solutions.
	IBestSolutionManager* bestSolManager;

	//! Manager of the local search operators.
	ILocalSearchManager* lsManager;

	//! The number of iterations since last weight recomputation.
	size_t nbIterationsWC;

	//! The current number of iterations.
	size_t nbIterations;

	//! The current number of iterations without improvement.
	size_t nbIterationsWithoutImprovement;

	//! The number of iterations without improvement of the current solution.
	size_t nbIterationsWithoutImprovementCurrent;

	//! The number of iterations without acceptation of a transition.
	size_t nbIterationsWithoutTransition;

	//! The number of iterations since the last call to a local search
	//! operator.
	size_t nbIterationsWithoutLocalSearch;

	//! The time the optimization process started.
	clock_t startingTime;

	//! A lower bound on the optimum solution cost.
	double lowerBound;

	//! A set containing the hash keys of the encountred solutions.
	std::set<long long> knownKeys;

	//! An object to compute some statistics about the solving process.
	Statistics stats;

	//! An object representing the status of the last iteration.
	ALNS_Iteration_Status status;

	//! A list of object that we need to update at the end of each iteration.
	std::vector<IUpdatable*> updatableStructures;

	std::string name;

public:
	//! Constructor.
	//! \param name the name of the instance.
	//! \param initialSolution the starting solution that is going to be optimized.
	//! \param acceptanceCrit the module that determine whether or not a new solution
	//! is accepted as the current solution.
	//! \param parameters the set of parameters to be use by the ALNS.
	//! \param opMan an operator manager.
	ALNS(std::string instanceName,
		ISolution& initialSolution,
		IAcceptanceModule& acceptanceCrit,
		 ALNS_Parameters& parameters,
		 AOperatorManager& opMan,
		 IBestSolutionManager& solMan,
		 ILocalSearchManager& lsMan);

	//! Destructor.
	virtual ~ALNS();

	//! This method launch the solving process.
	//! \return true if a feasible solution is found,
	//! false otherwise.
	bool solve();

	//! This method seeks if a solution is already known,
	//! if not it is added to the set of known solutions.
	//! \param sol the solution to be checked.
	//! \return true if the solution was unknown, false otherwise.
	bool checkAgainstKnownSolution(ISolution& sol);

	//! This method perform one iteration of the ALNS solving
	//! process.
	void performOneIteration();

	//! This method check whether or not the stopping criteria is met.
	bool isStoppingCriterionMet();

	//! Determine whether or not the new solution is better than the
	//! best known solution.
	bool isNewBest(ISolution* newSol);

	//! \return the number of known solutions.
	size_t getNumberKnownSolutions(){return knownKeys.size();};

	//! Determine whether or not the new solution should be accepted
	//! as the current solution.
	bool transitionCurrentSolution(ISolution* newSol);

	//! Return a pointer to the best known solution.
	IBestSolutionManager* getBestSolutionManager(){return bestSolManager;};

	//! Add an object to the list of object to be updated at the end of each
	//! iteration of the ALNS.
	//! \param up the updatable object to be added.
	void addUpdatable(IUpdatable& up){updatableStructures.push_back(&up);};
};

#endif /* ALNS_H_ */
