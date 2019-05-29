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


#ifndef SIMULATEDANNEALING_H_
#define SIMULATEDANNEALING_H_

#include "IAcceptanceModule.h"

class ISolution;
class ICoolingSchedule;
class ALNS_Iteration_Status;
class IBestSolutionManager;

/*!
 * \class SimulatedAnnealing.
 * \brief Use a simulated annealing principle to decide whether or not a new solution
 * should be accepted as the current solution.
 *
 * If you are interested by simulated annealing, please refer to:
 * Kirkpatrick, S.; Gelatt, C. D.; Vecchi, M. P. (1983). "Optimization by Simulated
 * Annealing". Science 220 (4598): 671–680
 * or
 * http://en.wikipedia.org/wiki/Simulated_annealing
 */

class SimulatedAnnealing: public IAcceptanceModule {
private:
	//! The cooling schedule to be use to compute the temperature each time it
	//! is needed.
	ICoolingSchedule* coolingSchedule;
public:
	//! Constructor.
	//! \param cs the cooling schedule to be used by the simulated annealing.
	SimulatedAnnealing(ICoolingSchedule& cs);

	//! Destructor.
	virtual ~SimulatedAnnealing();

	//! Compute if the newly created solution have to be accepted or not
	bool transitionAccepted(IBestSolutionManager& bestSolutionManager, ISolution& currentSolution, ISolution& newSolution, ALNS_Iteration_Status& status);

	virtual void startSignal();

};

#endif /* SIMULATEDANNEALING_H_ */
