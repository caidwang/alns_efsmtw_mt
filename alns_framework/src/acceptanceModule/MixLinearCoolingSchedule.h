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

#ifndef MIXLINEARCOOLINGSCHEDULE_H_
#define MIXLINEARCOOLINGSCHEDULE_H_

#include <time.h>
#include "ICoolingSchedule.h"

class ISolution;
class CoolingSchedule_Parameters;

/*!
 * \class MixLinearCoolingSchedule.
 * \brief A linear cooling schedule based on a mix of the total number of iterations
 * and the maximum running time.
 */

class MixLinearCoolingSchedule: public ICoolingSchedule {
public:
	//! Constructor.
	//! \param initSol the initial solution.
	//! \param csParam the cooling schedule parameters to be used.
	MixLinearCoolingSchedule(ISolution& initSol, CoolingSchedule_Parameters& csParam);

	//! Destructor.
	virtual ~MixLinearCoolingSchedule(){};

	//! Compute and return the current temperature.
	//! \return the current temperature.
	double getCurrentTemperature();


	virtual void startSignal();
private:
	//! The initial temperature.
	double startTemperature;

	//! The maximum run time.
	double runTime;

	//! The starting time.
	clock_t startingTime;

	//! The ending time.
	clock_t endingTime;

	//! The maximum number of iterations.
	size_t maxIt;

	//! The current iteration.
	size_t currentIt;
};

#endif /* MIXLINEARCOOLINGSCHEDULE_H_ */
