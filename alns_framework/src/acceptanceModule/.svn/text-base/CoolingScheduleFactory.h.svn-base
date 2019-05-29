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

#ifndef COOLINGSCHEDULEFACTORY_H_
#define COOLINGSCHEDULEFACTORY_H_

class ISolution;
class ICoolingSchedule;
class CoolingSchedule_Parameters;

/*!
 * \class CoolingScheduleFactory.
 * \brief This class instantiate cooling schedules.
 *
 */

class CoolingScheduleFactory {
public:
	//! Generate a cooling schedule using the initial solution and the cooling schedule parameters.
	//! \param sol the initial solution.
	//! \param param the parameters to be used to generate the cooling schedule.
	//! \return a pointer to a cooling schedule.
	static ICoolingSchedule* makeCoolingSchedule(ISolution& sol, CoolingSchedule_Parameters& param);
};

#endif /* COOLINGSCHEDULEFACTORY_H_ */
