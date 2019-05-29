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

#ifndef EXPONENTIALCOOLINGSCHEDULE_H_
#define EXPONENTIALCOOLINGSCHEDULE_H_

#include <time.h>
#include "ICoolingSchedule.h"

class ISolution;
class CoolingSchedule_Parameters;

/*!
 * \class ExponentialCoolingSchedule.
 * \brief An exponential cooling schedule based on a mix of the maximum running time
 * and the number of iterations.
 */

class ExponentialCoolingSchedule: public ICoolingSchedule {
public:
	//! Constructor.
	//! \param initSol the initial solution.
	//! \param csParam the cooling schedule parameters to be used.
	ExponentialCoolingSchedule(ISolution& initSol, CoolingSchedule_Parameters& csParam);

	//! Destructor.
	virtual ~ExponentialCoolingSchedule();

	//! Compute and return the current temperature.
	//! \return the current temperature.
	double getCurrentTemperature();

	//! This method is called when the optimization process start.
	virtual void startSignal();
private:
	//! The current iteration
	long long currentIt;

	//! The current iteration
	long long maximumIt;

	//! The time at which the algorithm started.
	clock_t startingTime;

	//! The time at which the algorithm should end.
	clock_t endingTime;

	//! The current threshold.
	int currentThreshold;

	//! The number of thresholds.
	int nbThresholds;

	//! The current temperature.
	double currentTemperature;

	//! The value multiplying the temperature between each threshold.
	double decreasingFactor;

	double runTime;
};

#endif /* EXPONENTIALCOOLINGSCHEDULE_H_ */
