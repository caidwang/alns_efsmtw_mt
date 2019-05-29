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
#include <stdlib.h>
#include <iostream>
#include <time.h>
#include <set>
#include "OperatorManager.h"
#include "ALNS_Parameters.h"
#include "AOperator.h"
#include "ARepairOperator.h"
#include "ADestroyOperator.h"
#include "ALNS_Iteration_Status.h"
#include "../statistics/Statistics.h"

OperatorManager::OperatorManager(ALNS_Parameters& param) {
	parameters = &param;
	sumWeightsRepair = 0;
	sumWeightsDestroy = 0;
	noise = false;
}

OperatorManager::~OperatorManager() {
	// Nothing to be done.
}

void OperatorManager::recomputeWeight(AOperator& op, double& sumW)
{
	double prevWeight = op.getWeight();
	sumW -= prevWeight;
	double currentScore = op.getScore();
	size_t nbCalls = op.getNumberOfCallsSinceLastEvaluation();
	double newWeight = (1-parameters->getRho())*prevWeight + parameters->getRho()*(static_cast<double>(nbCalls)/static_cast<double>(parameters->getTimeSegmentsIt()))*currentScore;
	// We ensure that the weight is within the bounds.
	if(newWeight > parameters->getMaximumWeight())
	{
		newWeight = parameters->getMaximumWeight();
	}
	if(newWeight < parameters->getMinimumWeight())
	{
		newWeight = parameters->getMinimumWeight();
	}

	sumW += newWeight;
	op.setWeight(newWeight);
	op.resetScore();
	op.resetNumberOfCalls();
}

void OperatorManager::recomputeWeights()
{
	// Retrieval of the number of calls for the statistics module.
	std::vector<size_t>* nbCalls = new std::vector<size_t>();
	for(size_t i = 0; i < repairOperators.size(); i++)
	{
		nbCalls->push_back(repairOperators[i]->getNumberOfCallsSinceLastEvaluation());
	}
	for(size_t i = 0; i < destroyOperators.size(); i++)
	{
		nbCalls->push_back(destroyOperators[i]->getNumberOfCallsSinceLastEvaluation());
	}


	// Weight recomputation for repair operators.
	for(size_t i = 0; i < repairOperators.size(); i++)
	{
		recomputeWeight(dynamic_cast<AOperator&>(*(repairOperators[i])),sumWeightsRepair);
	}

	//! Weight recomputation for destroy operators.
	for(size_t i = 0; i < destroyOperators.size(); i++)
	{
		recomputeWeight(dynamic_cast<AOperator&>(*(destroyOperators[i])),sumWeightsDestroy);
	}

	// Retrieval of the weights for the statistics module.
	std::vector<double>* weightsStats = new std::vector<double>();
	for(size_t i = 0; i < repairOperators.size(); i++)
	{
		weightsStats->push_back(repairOperators[i]->getWeight());
	}
	for(size_t i = 0; i < destroyOperators.size(); i++)
	{
		weightsStats->push_back(destroyOperators[i]->getWeight());
	}

	stats->addOperatorEntry(weightsStats,nbCalls);
}

void OperatorManager::startSignal()
{
	std::vector<std::string>* names = new std::vector<std::string>();
	for(size_t i = 0; i < repairOperators.size(); i++)
	{
		names->push_back(repairOperators[i]->getName());
	}
	for(size_t i = 0; i < destroyOperators.size(); i++)
	{
		names->push_back(destroyOperators[i]->getName());
	}
	stats->addOperatorsNames(names);
}

AOperator& OperatorManager::selectOperator(std::vector<AOperator*>& vecOp, double sumW)
{
	double randomVal = static_cast<double>(rand())/static_cast<double>(RAND_MAX);
	double randomWeightPos = randomVal*sumW;
	double cumulSum = 0;
	for(size_t i = 0; i < vecOp.size(); i++)
	{
		cumulSum += vecOp[i]->getWeight();
		if(cumulSum >= randomWeightPos)
		{
			if(noise)
			{
				vecOp[i]->setNoise();
			}
			else
			{
				vecOp[i]->unsetNoise();
			}
			vecOp[i]->increaseNumberOfCalls();
			return *(vecOp[i]);
		}
	}
	assert(false);
	return *(vecOp.back());
}

ADestroyOperator& OperatorManager::selectDestroyOperator()
{
	return dynamic_cast<ADestroyOperator&>(selectOperator(destroyOperators,sumWeightsDestroy));
}

ARepairOperator& OperatorManager::selectRepairOperator()
{
	return dynamic_cast<ARepairOperator&>(selectOperator(repairOperators,sumWeightsRepair));
}

void OperatorManager::addRepairOperator(ARepairOperator& repairOperator)
{
	//TODO find out why the set.find()==set.end() does not work

	bool ok = true;
	for(size_t i = 0; i < parameters->getForbidenOperators().size() && ok; i++)
	{
		if(parameters->getForbidenOperators()[i] == repairOperator.getName())
		{
			std::cout << repairOperator.getName().c_str() << " forbidden" << std::endl;
			ok = false;
		}
	}
	if(ok)
	{
		repairOperators.push_back(&repairOperator);
		sumWeightsRepair += repairOperator.getWeight();
	}



}

void OperatorManager::addDestroyOperator(ADestroyOperator& destroyOperator)
{
	//TODO find out why the set.find()==set.end() does not work
	bool ok = true;
	for(size_t i = 0; i < parameters->getForbidenOperators().size() && ok; i++)
	{
		if(parameters->getForbidenOperators()[i] == destroyOperator.getName())
		{
			std::cout << destroyOperator.getName().c_str() << " forbidden" << std::endl;
			ok = false;
		}
	}
	if(ok)
	{
		destroyOperators.push_back(&destroyOperator);
		sumWeightsDestroy += destroyOperator.getWeight();
	}

}

void OperatorManager::sanityChecks()
{
	assert(!repairOperators.empty());
	assert(!destroyOperators.empty());
	assert(sumWeightsDestroy>0);
	assert(sumWeightsRepair>0);
}

void OperatorManager::updateScores(ADestroyOperator& des, ARepairOperator& rep, ALNS_Iteration_Status& status)
{
	if(parameters->getNoise())
	{
		double randNoise = static_cast<double>(rand())/RAND_MAX;
		noise = (randNoise>parameters->getProbabilityOfNoise());
	}

	if(status.getNewBestSolution() == ALNS_Iteration_Status::TRUE)
	{
		rep.setScore(rep.getScore()+parameters->getSigma1());
		des.setScore(des.getScore()+parameters->getSigma1());
	}

	if(status.getImproveCurrentSolution() == ALNS_Iteration_Status::TRUE)
	{
		rep.setScore(rep.getScore()+parameters->getSigma2());
		des.setScore(des.getScore()+parameters->getSigma2());
	}

	if(status.getImproveCurrentSolution() == ALNS_Iteration_Status::FALSE
			&& status.getAcceptedAsCurrentSolution() == ALNS_Iteration_Status::TRUE
			&& status.getAlreadyKnownSolution() == ALNS_Iteration_Status::FALSE)
	{
		rep.setScore(rep.getScore()+parameters->getSigma3());
		des.setScore(des.getScore()+parameters->getSigma3());
	}
}
