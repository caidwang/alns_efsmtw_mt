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

#ifndef TIMELINEARCOOLINGSCHEDULE_H_
#define TIMELINEARCOOLINGSCHEDULE_H_

#include <time.h>
#include "ICoolingSchedule.h"

class ISolution;
class CoolingSchedule_Parameters;

/*!
 * \class TimeLinearCoolingSchedule.
 * \brief A linear cooling schedule based on the maximum running time.
 */

class TimeLinearCoolingSchedule: public ICoolingSchedule {
public:
	//! Constructor.
	//! \param initSol the initial solution.
	//! \param csParam the cooling schedule parameters to be used.
	TimeLinearCoolingSchedule(ISolution& initSol, CoolingSchedule_Parameters& csParam);

	//! Destructor.
	virtual ~TimeLinearCoolingSchedule(){};

	//! Compute and return the current temperature.
	//! \return the current temperature.
	double getCurrentTemperature();

	virtual void startSignal();

private:
	//! The starting temperature.
	double startTemperature;

	//! The starting time.
	clock_t startingTime;

	//! The ending time.
	clock_t endingTime;

	//! The maximum run time.
	double runTime;

};

#endif /* TIMELINEARCOOLINGSCHEDULE_H_ */
