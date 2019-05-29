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

#ifndef IBESTSOLUTIONMANAGER_H_
#define IBESTSOLUTIONMANAGER_H_

#include <list>

class ISolution;
class ALNS_Iteration_Status;

class IBestSolutionManager
{
public:
	//! This method evaluate if a solution is a new best solution, and adds it to the
	//! best solution pool in this case.
	//! \param sol the solution to be tested.
	//! \return true if the solution is a new best solution, false otherwise.
	virtual bool isNewBestSolution(ISolution& sol)=0;

	//! Return a pointer to a best solution.
	virtual std::list<ISolution*>::iterator begin()=0;

	//! Return a pointer to a best solution.
	virtual std::list<ISolution*>::iterator end()=0;

	//! This function take care of reloading the best known
	//! solution, as the current solution, if needed.
	//! \param currSol a pointer to the current solution.
	//! \param status the status of the current iteration.
	//! \return a pointer to the current solution.
	virtual ISolution* reloadBestSolution(ISolution* currSol, ALNS_Iteration_Status& status)=0;
};


#endif /* IBESTSOLUTIONMANAGER_H_ */
