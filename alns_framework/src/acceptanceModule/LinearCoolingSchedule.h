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

#ifndef LINEARCOOLINGSCHEDULE_H_
#define LINEARCOOLINGSCHEDULE_H_

#include <cstring>
#include "ICoolingSchedule.h"


class ISolution;
class CoolingSchedule_Parameters;

/*!
 * \class LinearCoolingSchedule.
 * \brief A linear cooling schedule based on the total number of iterations.
 */

class LinearCoolingSchedule: public ICoolingSchedule {
private:
	//! The current temperature.
	double currentTemperature;

	//! The amount to remove at each temperature recomputation.
	double amountRemove;
public:
	//! Constructor.
	//! \param initSol the initial solution.
	//! \param csParam the cooling schedule parameters.
	//! \param nbIterations the number of iterations to be performed.
	LinearCoolingSchedule(ISolution& initSol, CoolingSchedule_Parameters& csParam, size_t nbIterations);

	//! Constructor.
	//! \param startingTemperature the initial temperature.
	//! \param nbIterations the number of iterations to be performed.
	LinearCoolingSchedule(double startingTemperature, size_t nbIterations);

	//! Destructor.
	virtual ~LinearCoolingSchedule();

	//! Compute and return the current temperature.
	//! \return the current temperature.
	double getCurrentTemperature();

	void startSignal(){};
};

#endif /* LINEARCOOLINGSCHEDULE_H_ */
