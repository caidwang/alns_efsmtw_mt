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

#include "SimpleLocalSearchManager.h"
#include "../alns/ALNS_Iteration_Status.h"
#include "../alns/ALNS_Parameters.h"
#include "../alns/ISolution.h"
#include "ILocalSearch.h"


bool SimpleLocalSearchManager::useLocalSearch(ISolution& sol, ALNS_Iteration_Status& status)
{
	if(status.getNewBestSolution()!=ALNS_Iteration_Status::TRUE
		|| status.getAcceptedAsCurrentSolution()!=ALNS_Iteration_Status::UNKNOWN)
	{
		return false;
	}
	else
	{
		status.setLocalSearchUsed(ALNS_Iteration_Status::TRUE);
		bool improvement;
		do
		{
			improvement = false;
			for(size_t i = 0; i < localSearchOperators.size(); i++)
			{
				improvement = localSearchOperators[i]->performLocalSearch(sol)||improvement;
			}
		}while(improvement);
		if(improvement)
		{
			status.setImproveByLocalSearch(ALNS_Iteration_Status::TRUE);
			return true;
		}
		else
		{
			status.setImproveByLocalSearch(ALNS_Iteration_Status::FALSE);
			return false;
		}
	}
}

void SimpleLocalSearchManager::addLocalSearchOperator(ILocalSearch& ls)
{
	//TODO find out why the set.find() == set.end() does not work.
	bool ok = true;
	for(size_t i=0; i< param->getForbidenLsOperators().size() && ok; i++)
	{
		if(param->getForbidenLsOperators()[i] == ls.getName())
		{
			std::cout << "NO " << ls.getName() << std::endl;
			ok = false;
		}
	}
	if(ok)
	{
		localSearchOperators.push_back(&ls);
	}

}
