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

#include "SimpleBestSolutionManager.h"
#include <list>
#include "ISolution.h"
#include "ALNS_Iteration_Status.h"
#include "ALNS_Parameters.h"



using namespace std;

SimpleBestSolutionManager::SimpleBestSolutionManager(ALNS_Parameters& param) {
	parameters = &param;
}

SimpleBestSolutionManager::~SimpleBestSolutionManager() {
	for(list<ISolution*>::iterator it = bestSols.begin(); it != bestSols.end(); it++)
	{
		delete (*it);
	}
}

bool SimpleBestSolutionManager::isNewBestSolution(ISolution& sol)
{
	for(list<ISolution*>::iterator it = bestSols.begin(); it != bestSols.end(); it++)
	{
		ISolution& currentSol = *(*it);
		if(currentSol<sol)
		{
			return false;
		}
		else if(sol<currentSol)
		{
			delete *it;
			it = bestSols.erase(it);
			if(it == bestSols.end())
			{
				break;
			}
		}
		else if(currentSol.getHash() == sol.getHash())
		{
			return false;
		}
	}
	ISolution* copy = sol.getCopy();
	bestSols.push_back(copy);
	return true;
}

ISolution* SimpleBestSolutionManager::reloadBestSolution(ISolution* currSol, ALNS_Iteration_Status& status)
{
	if(status.getNbIterationWithoutImprovementSinceLastReload() > 0 &&
	   ((status.getNbIterationWithoutImprovementSinceLastReload() % parameters->getReloadFrequency()) == 0))
	{
		status.setNbIterationWithoutImprovementSinceLastReload(0);
		delete currSol;
		return bestSols.back()->getCopy();
	}
	else
	{
		return currSol;
	}
}
