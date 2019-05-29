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

#include "ExponentialCoolingSchedule.h"
#include <algorithm>
#include <math.h>
#include <iostream>
#include "../alns/ISolution.h"
#include "CoolingSchedule_Parameters.h"

ExponentialCoolingSchedule::ExponentialCoolingSchedule(ISolution& initSol, CoolingSchedule_Parameters& csParam) {
	// The fields are instantiated to default values.
	this->maximumIt = csParam.maxIt;
	this->currentIt = 0;
	this->currentThreshold = 0;
	this->nbThresholds = csParam.nbThresholds;
	this->decreasingFactor = csParam.expPercentageKept;
	this->runTime = csParam.maxRT;
	currentTemperature = (csParam.setupPercentage*initSol.getPenalizedObjectiveValue())/(-log(0.5));
}

ExponentialCoolingSchedule::~ExponentialCoolingSchedule() {
	// Nothing to be done
}

double ExponentialCoolingSchedule::getCurrentTemperature()
{
	currentIt++;
	clock_t currentTime = clock();
	double percentageTime = ((double)(currentTime - startingTime))/(endingTime- startingTime);
	double percentageIt = ((double)currentIt)/maximumIt;
	double percentageM = percentageTime;
	if(percentageIt > percentageM)
	{
		percentageM = percentageIt;
	}
	int aimedThreshold = (int)(percentageM*nbThresholds);
	while(currentThreshold < aimedThreshold)
	{
		currentThreshold++;
		currentTemperature *= decreasingFactor;
	}
	return currentTemperature;
}

void ExponentialCoolingSchedule::startSignal()
{
	startingTime = clock();
	endingTime = startingTime + CLOCKS_PER_SEC*runTime;
}
