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

#ifndef COOLINGSCHEDULE_PARAMETERS_H_
#define COOLINGSCHEDULE_PARAMETERS_H_

#include <cstring>
#include <iostream>

class ALNS_Parameters;

/*!
 * \class CoolingSchedule_Parameters.
 * \brief This class represent parameters to be used to instantiate cooling schedules.
 */

class CoolingSchedule_Parameters
{
public:

	//! Enumeration representing the various kind of cooling schedules that can be handled.
	enum CSKind{
		Linear_it,
		Linear_time,
		Linear_mix,
		Exponential_it,
		Exponential_time,
		Exponential_mix
	};

	//! Constructor.
	CoolingSchedule_Parameters(ALNS_Parameters& alnsParam);

	//! Default Constructor.
	CoolingSchedule_Parameters();

	//! Copy constructor.
	CoolingSchedule_Parameters(CoolingSchedule_Parameters& p);

	//! Destructor.
	~CoolingSchedule_Parameters();

	//! Perform some sanity checks on the values of the parameters.
	void sanityChecks();

	//! Load the parameters from a text file.
	//! \param path the path to the text file containing the parameters.
	void loadParameters(std::string path);

	//! Load the parameters from a text file.
	//! \param path the path to the text file containing the parameters.
	void loadXMLParameters(std::string path);

	//! The kind of cooling schedule to be used.
	CSKind kind;

	//! In case of the use of an exponential cooling schedule, the percentage of
	//! the current temperature kept at each temperature recomputation.
	double expPercentageKept;

	//! To determine the starting temperature, how worse than the initial solution can
	//! a solution be to have a 50% chance of being accepted at the starting temperature.
	double setupPercentage;

	//! The number of temperature recomputations during the optimization process.
	size_t nbThresholds;

	//! The maximum number of iterations.
	size_t maxIt;

	//! The maximum running time.
	size_t maxRT;


};


#endif /* COOLINGSCHEDULE_PARAMETERS_H_ */
