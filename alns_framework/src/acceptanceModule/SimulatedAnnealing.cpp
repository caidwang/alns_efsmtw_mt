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
#include <cmath>
#include <stdlib.h>
#include <time.h>
#include "SimulatedAnnealing.h"
#include "ICoolingSchedule.h"
#include "../alns/ISolution.h"
#include "../alns/ALNS_Iteration_Status.h"
#include "../alns/IBestSolutionManager.h"

SimulatedAnnealing::SimulatedAnnealing(ICoolingSchedule& cs) {
	coolingSchedule = &cs;
}

SimulatedAnnealing::~SimulatedAnnealing() {
	delete coolingSchedule;
}

bool SimulatedAnnealing::transitionAccepted(IBestSolutionManager& bestSolutionManager, ISolution& currentSolution, ISolution& newSolution, ALNS_Iteration_Status& status)
{
	double temperature = coolingSchedule->getCurrentTemperature();
	if(newSolution < currentSolution)
	{
		return true;
	}
	else
	{
		double difference = newSolution.getPenalizedObjectiveValue() - currentSolution.getPenalizedObjectiveValue();
		double randomVal = static_cast<double>(rand())/static_cast<double>(RAND_MAX);
		return (exp(-1*difference/temperature)>randomVal);
	}
}

void SimulatedAnnealing::startSignal()
{
	coolingSchedule->startSignal();
}
