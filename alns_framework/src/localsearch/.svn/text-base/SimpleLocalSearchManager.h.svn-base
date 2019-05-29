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

#ifndef SIMPLELOCALSEARCHMANAGER_H_
#define SIMPLELOCALSEARCHMANAGER_H_

#include <vector>
#include "ILocalSearchManager.h"


class ILocalSearch;
class ISolution;
class ALNS_Parameters;

class SimpleLocalSearchManager: public ILocalSearchManager {
public:

	SimpleLocalSearchManager(ALNS_Parameters& parameters){param = &parameters;};

	virtual ~SimpleLocalSearchManager(){};

	//! \param sol the solution to be improved.
	//! \param status the status of the alns iteration.
	//! \return true if the solution has been improved.
	virtual bool useLocalSearch(ISolution& sol, ALNS_Iteration_Status& status);

	//! Add a local search operator to the manager.
	void addLocalSearchOperator(ILocalSearch& ls);


	virtual void startSignal(){};
private:
	//! A vector containing the local search operators managed by the current instance.
	std::vector<ILocalSearch*> localSearchOperators;

	//! Parameters of the ALNS.
	ALNS_Parameters* param;
};

#endif /* SIMPLELOCALSEARCHMANAGER_H_ */
