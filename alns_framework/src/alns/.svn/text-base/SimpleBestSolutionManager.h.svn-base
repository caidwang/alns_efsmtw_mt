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

#ifndef SIMPLEBESTSOLUTIONMANAGER_H_
#define SIMPLEBESTSOLUTIONMANAGER_H_

#include "IBestSolutionManager.h"
#include <list>

class ISolution;
class ALNS_Iteration_Status;
class ALNS_Parameters;

class SimpleBestSolutionManager: public IBestSolutionManager {
public:
	SimpleBestSolutionManager(ALNS_Parameters& param);

	virtual ~SimpleBestSolutionManager();

	virtual bool isNewBestSolution(ISolution& sol);

	//! Return a pointer to a best solution.
	std::list<ISolution*>::iterator begin(){return bestSols.begin();};

	//! Return a pointer to a best solution.
	std::list<ISolution*>::iterator end(){return bestSols.end();};

	//! This function take care of reloading the best known
	//! solution, as the current solution, if needed.
	//! \param currSol a pointer to the current solution.
	//! \param status the status of the current iteration.
	//! \return a pointer to the current solution.
	virtual ISolution* reloadBestSolution(ISolution* currSol, ALNS_Iteration_Status& status);

	//! Simple getter.
	std::list<ISolution*>& getBestSols(){return bestSols;};
private:
	std::list<ISolution*> bestSols;

	ALNS_Parameters* parameters;

};

#endif /* SIMPLEBESTSOLUTIONMANAGER_H_ */
