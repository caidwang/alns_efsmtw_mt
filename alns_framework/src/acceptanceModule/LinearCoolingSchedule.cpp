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

#include <assert.h>
#include <math.h>
#include "LinearCoolingSchedule.h"
#include "CoolingSchedule_Parameters.h"
#include "../alns/ISolution.h"

LinearCoolingSchedule::LinearCoolingSchedule(ISolution& initSol, CoolingSchedule_Parameters& csParam, size_t nbIterations) {
	currentTemperature = (csParam.setupPercentage*initSol.getPenalizedObjectiveValue())/(-log(0.5));
	amountRemove = currentTemperature/static_cast<double>(nbIterations);

}

LinearCoolingSchedule::LinearCoolingSchedule(double startingTemperature, size_t nbIterations) {
	assert(nbIterations>0);
	assert(startingTemperature>=0);
	currentTemperature = startingTemperature;
	amountRemove = startingTemperature/static_cast<double>(nbIterations);

}

LinearCoolingSchedule::~LinearCoolingSchedule() {
	// Nothing to be done.
}

double LinearCoolingSchedule::getCurrentTemperature()
{
	currentTemperature-= amountRemove;
	if(currentTemperature < 0)
	{
		currentTemperature = 0;
	}
	assert(currentTemperature>=0);
	return currentTemperature;
}
