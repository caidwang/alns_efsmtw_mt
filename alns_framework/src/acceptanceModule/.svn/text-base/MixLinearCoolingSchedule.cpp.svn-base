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
#include <algorithm>
#include <math.h>
#include <iostream>
#include "MixLinearCoolingSchedule.h"
#include "../alns/ISolution.h"
#include "CoolingSchedule_Parameters.h"

using namespace std;

MixLinearCoolingSchedule::MixLinearCoolingSchedule(ISolution& initSol, CoolingSchedule_Parameters& csParam) {
	startTemperature = (csParam.setupPercentage*initSol.getPenalizedObjectiveValue())/(-log(0.5));
	runTime = csParam.maxRT;
	maxIt = csParam.maxIt;
	currentIt = 0;
}

double MixLinearCoolingSchedule::getCurrentTemperature()
{
	double propIt = (1 - static_cast<double>(currentIt++)/static_cast<double>(maxIt));
	double propTime = static_cast<double>(endingTime-clock())/static_cast<double>(endingTime-startingTime);
	double prop = min(propIt,propTime);
	double returnT = startTemperature*prop;
	cout << returnT << endl;
	return returnT;
}

void MixLinearCoolingSchedule::startSignal()
{
	startingTime = clock();
	endingTime = startingTime + CLOCKS_PER_SEC*runTime;
}
