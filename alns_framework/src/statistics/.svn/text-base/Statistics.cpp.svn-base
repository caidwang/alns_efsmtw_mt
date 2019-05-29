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

#include <iostream>
#include <fstream>
#include <vector>
#include "Statistics.h"

using namespace std;

Statistics::~Statistics()
{
	for(size_t i = 0; i < nbCalls.size(); i++)
	{
		delete nbCalls[i];
		delete weights[i];
	}
	delete operatorNames;
}

void Statistics::addEntry(double timeStamp,
				  size_t iteration,
				  std::string destroyName,
				  std::string recreateName,
				  double newCost,
				  double currentCost,
				  double bestCost,
				  int cumKs)
{
	timeStamps.push_back(timeStamp);
	iterations.push_back(iteration);
	destroyNames.push_back(destroyName);
	recreateNames.push_back(recreateName);
	newCosts.push_back(newCost);
	currentCosts.push_back(currentCost);
	bestCosts.push_back(bestCost);
	cumulativeKnownSolutions.push_back(cumKs);
}

void Statistics::addOperatorEntry(std::vector<double>* weight,
					  std::vector<size_t>* calls)
{
	timeStampsOperators.push_back(static_cast<double>(clock()-start)/CLOCKS_PER_SEC);
	weights.push_back(weight);
	nbCalls.push_back(calls);
}

void Statistics::generateStatsFile(std::string pathGlob, std::string pathOp)
{
	ofstream myfile;
	myfile.open(pathGlob.c_str(), ios::out | ios::trunc);
	myfile << "iterations\ttimeStamps\tDestroy Operator\tRecreate Operator\tNew Cost\tCurrent Cost\tBest Cost\tCumulative Known Solutions\n";
	for(size_t i = 0; i < iterations.size(); i++)
	{
		myfile << iterations[i] << "\t" << timeStamps[i] << "\t" << destroyNames[i] << "\t" << recreateNames[i] << "\t" << newCosts[i] << "\t" << currentCosts[i] << "\t" << bestCosts[i] << "\t" << cumulativeKnownSolutions[i] << "\n";
	}
	myfile.close();

	ofstream myfileOp;
	myfileOp.open(pathOp.c_str(), ios::out | ios::trunc);
	myfileOp << "time stamp\t";
	for(size_t i = 0; i < operatorNames->size(); i++)
	{
		myfileOp << "weight " << operatorNames->at(i) << "\tCalls" << operatorNames->at(i) << "\t";
	}
	myfileOp << "\n";

	for(size_t i = 0; i < nbCalls.size(); i++)
	{
		myfileOp << timeStampsOperators[i] << "\t";
		for(size_t j = 0; j < operatorNames->size(); j++)
		{
			myfileOp << weights[i]->at(j) << "\t" << nbCalls[i]->at(j) << "\t";
		}
		myfileOp << "\n";
	}
	myfileOp.close();
}
