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

#ifndef IACCEPTANCEMODULE_H_
#define IACCEPTANCEMODULE_H_

#include <iostream>
#include "../alns/ISolution.h"
#include "../alns/ALNS_Iteration_Status.h"
#include "../alns/IBestSolutionManager.h"

/*!
 * \class IAcceptanceModule.
 * \brief This is an interface to define acceptance modules within the ALNS.
 */

class IAcceptanceModule
{
public:
	//! Indicate if the new created solution have to be accepted or not
	//! \param bestSolutionManager a reference to the best solution manager.
	//! \param currentSolution current solution.
	//! \param newSolution new solution.
	//! \param status the status of the current alns iteration.
	//! \return true if the transition is accepted, false otherwise.
	virtual bool transitionAccepted(IBestSolutionManager& bestSolutionManager, ISolution& currentSolution, ISolution& newSolution, ALNS_Iteration_Status& status) = 0;

	//! Some Acceptance modules needs to initialize some variable
	//! only when the solver actualy starts working. In this case
	//! you should override this method.
	virtual void startSignal(){};
};


#endif /* ACCEPTANCEMODULE_H_ */
