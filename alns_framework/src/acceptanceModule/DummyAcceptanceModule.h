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

#ifndef DUMMYACCEPTANCEMODULE_H_
#define DUMMYACCEPTANCEMODULE_H_

#include "IAcceptanceModule.h"

class ISolution;
class ALNS_Iteration_Status;
class IBestSolutionManager;

/*!
 * \class DummyAcceptanceModule.
 * \brief This module accept any solution as the current solution.
 * \details It can be used when the diversification is ensured through an other mechanism.
 * For example when the operators embbed some tabu criterion to prevent them from rebuilding
 * the previous solution.
 */

class DummyAcceptanceModule: public IAcceptanceModule {
public:
	//! Constructor.
	DummyAcceptanceModule();
	//! Destructor.
	virtual ~DummyAcceptanceModule();
	//! Compute if the newly created solution have to be accepted or not
	bool transitionAccepted(IBestSolutionManager& bestSolutionManager,
							ISolution& currentSolution,
							ISolution& newSolution,
							ALNS_Iteration_Status& status);

};

#endif /* DUMMYACCEPTANCEMODULE_H_ */
