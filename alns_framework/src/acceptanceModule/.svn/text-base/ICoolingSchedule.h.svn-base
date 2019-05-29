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

#ifndef ICOOLINGSCHEDULE_H_
#define ICOOLINGSCHEDULE_H_

#include <iostream>

/*!
 * \class ICoolingSchedule.
 * \brief This is an interface to define cooling schedule to be used by a simulated annealing.
 */

class ICoolingSchedule
{
public:
	//! \return the current temperature.
	virtual double getCurrentTemperature()=0;

	//! This method should be called when the optimization
	//! process start. The cooling schedules that actually need
	//! this should override this method.
	virtual void startSignal(){};
};


#endif /* ICOOLINGSCHEDULE_H_ */
