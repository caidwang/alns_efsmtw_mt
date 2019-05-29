/* ALNS-framework - a solver for temporal constraint problems
 *
 * Copyright (C) 2013 Renaud Masson
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
 * The Original Code is the ALNS-framework library.
 *
 * File name:
 *  ParameterTuning.cpp
 * Contributor(s):
 *	Renaud Masson
 */

#define _GLIBCXX_USE_NANOSLEEP

#include <fstream>
#include <cfloat>
#include <cstdlib>
#include <ctime>
#include <iomanip>
#include <chrono>
#include <thread>
#include "ParameterTuning.h"
#include "../ALNS_inc.h"
#include <sstream>

using namespace std;


ParameterTuning::~ParameterTuning() {
	// TODO Auto-generated destructor stub
}

std::string ParameterTuning::buildHash(ALNS_Parameters_optimizable& ap,
		CoolingSchedule_Parameters_Optimizable& cp)
{
	stringstream ss;
	ss << ap.getHash() << "-" << cp.getHash();
	return ss.str();
}

void ParameterTuning::runInstance(ALNS_Parameters_optimizable* ap,
		CoolingSchedule_Parameters_Optimizable* cp, ISolution* s)
{
	availableThreads--;
	ALNS* solver = solverBuilder->buildSolver(ap,cp,s);
	solver->solve();
	ISolution* best = *(solver->getBestSolutionManager()->begin());
	currentScores.push_back(best->getObjectiveValue());
	delete solver;
	delete ap;
	delete cp;
	delete s;
	availableThreads++;
}

void ParameterTuning::modifyParameters(ALNS_Parameters_optimizable* ap,
		CoolingSchedule_Parameters_Optimizable* cp) {
	int randInt = rand()%(ap->getNbTunableParameters()+cp->getNbTunableParameters());
	if(randInt < ap->getNbTunableParameters())
	{
		ap->modifyOneParameter();
	}
	else
	{
		cp->modifyOneParameter();
	}
}

double ParameterTuning::evaluateConfiguration(ALNS_Parameters_optimizable& ap,
		CoolingSchedule_Parameters_Optimizable& cp)
{
	string hash = buildHash(ap,cp);
	map<string,double>::iterator it = scores.find(hash);
	// If this configuration has already been evaluated we just access its cost.
	if(it == scores.end())
	{
		if(ap.getMinDestroyPerc() > ap.getMaxDestroyPerc()
			|| ap.getMinimumWeight() > ap.getMaximumWeight())
		{
			scores.insert(pair<string,double>(hash,DBL_MAX));
			return DBL_MAX;
		}
		else
		{
			thread** t = new thread*[nbRunPerInstance*problemSet.size()];
			for(int i = 0; i < nbRunPerInstance; ++i)
			{
				for(int j = 0; j < problemSet.size(); ++j)
				{
					bool launched = false;
					while(!launched)
					{
						std::this_thread::sleep_for(std::chrono::milliseconds(100));
						if(availableThreads > 0)
						{
							t[nbRunPerInstance*problemSet.size() + j] = new thread(&ParameterTuning::runInstance,this,new ALNS_Parameters_optimizable(ap), new CoolingSchedule_Parameters_Optimizable(cp), problemSet[j]->getCopy());
							launched = true;
						}
					}
				}
			}

			for(int i = 0; i < nbRunPerInstance*problemSet.size(); ++i)
			{
				// Means that the next command should not be executed before
				// this thread has finished its work.
				t[i]->join();
			}

			for(int i = 0; i < nbRunPerInstance*problemSet.size(); ++i)
			{
				delete t[i];
			}
			delete[] t;

			double sumScores = 0;
			for(int i = 0; i < currentScores.size(); ++i)
			{
				sumScores += currentScores[i];
			}
			currentScores.clear();
			scores.insert(pair<string,double>(hash,sumScores));
			return sumScores;
		}
	}
	else
	{
		return it->second;
	}
}

ParameterTuning::ParameterTuning(ALNS_Parameters_optimizable* alnsParams,
		CoolingSchedule_Parameters_Optimizable* amParams,
		std::vector<ISolution*> trainingSet, long long runTime,
		int nbPerInstance, int nbThreads, ISolverBuilder* solverFactory) : alnsParams(alnsParams), amParams(amParams), maximumRuntime(runTime), nbRunPerInstance(nbPerInstance), maximumNbThreads(nbThreads), availableThreads(nbThreads), solverBuilder(solverFactory)
{
	bestAlnsParams = new ALNS_Parameters_optimizable(*alnsParams);
	bestAmParams = new CoolingSchedule_Parameters_Optimizable(*amParams);
	for(int i = 0; i < trainingSet.size(); ++i)
	{
		problemSet.push_back(trainingSet[i]);
	}
	bestScore = DBL_MAX;
}

void ParameterTuning::writeBestParameters(std::string path)
{
	ofstream os;
	os.open(path.c_str(), ios::out);
	os << "<?xml version=\"1.0\" encoding=\"utf-8\"?>" << endl;
	os << "<Parameters>" << endl;
	os << setw(2) << "<ALNS>" << endl;
	os << setw(4) << "<MaxNbIterations>" << bestAlnsParams->getMaxNbIterations() << "</MaxNbIterations>" << endl;
	os << setw(4) << "<MaxRunTime>" << bestAlnsParams->getMaxRunningTime() << "</MaxRunTime>" << endl;
	os << setw(4) << "<MaxNbIterationsNoImp>" << bestAlnsParams->getMaxNbIterationsNoImp() << "</MaxNbIterationsNoImp>" << endl;
	switch(bestAlnsParams->getStopCrit())
	{
	case ALNS_Parameters::MAX_IT:
		os << setw(4) << "<StoppingCriterion>MAX_IT</StoppingCriterion>" << endl;
		break;
	case ALNS_Parameters::MAX_RT:
		os << setw(4) << "<StoppingCriterion>MAX_RT</StoppingCriterion>" << endl;
		break;
	case ALNS_Parameters::MAX_IT_NO_IMP:
		os << setw(4) << "<StoppingCriterion>MAX_IT_NO_IMP</StoppingCriterion>" << endl;
		break;
	case ALNS_Parameters::ALL:
		os << setw(4) << "<StoppingCriterion>ALL</StoppingCriterion>" << endl;
		break;
	}
	if(bestAlnsParams->getNoise())
	{
		os << setw(4) << "<Noise>true</Noise>" << endl;
	}
	else
	{
		os << setw(4) << "<Noise>false</Noise>" << endl;
	}
	os << setw(4) << "<TimeSegmentIt>" << bestAlnsParams->getTimeSegmentsIt() << "</TimeSegmentIt>" << endl;
	os << setw(4) << "<NbItBeforeReinit>" << bestAlnsParams->getNbItBeforeReinit() << "</NbItBeforeReinit>" << endl;
	os << setw(4) << "<Sigma1>" << bestAlnsParams->getSigma1() << "</Sigma1>" << endl;
	os << setw(4) << "<Sigma2>" << bestAlnsParams->getSigma2() << "</Sigma2>" << endl;
	os << setw(4) << "<Sigma3>" << bestAlnsParams->getSigma3() << "</Sigma3>" << endl;
	os << setw(4) << "<Rho>" << bestAlnsParams->getRho() << "</Rho>" << endl;
	os << setw(4) << "<MinimumWeight>" << bestAlnsParams->getMinimumWeight() << "</MinimumWeight>" << endl;
	os << setw(4) << "<MaximumWeight>" << bestAlnsParams->getMaximumWeight() << "</MaximumWeight>" << endl;
	os << setw(4) << "<ProbabilityOfNoise>" << bestAlnsParams->getProbabilityOfNoise() << "</ProbabilityOfNoise>" << endl;
	os << setw(4) << "<AcKind>SA</AcKind>" << endl;
	os << setw(4) << "<ReloadFrequency>" << bestAlnsParams->getReloadFrequency() << "</ReloadFrequency>" << endl;
	os << setw(4) << "<LogFrequency>" << bestAlnsParams->getLogFrequency() << "</LogFrequency>" << endl;
	os << setw(4) << "<StatsGlobPath>" << bestAlnsParams->getStatsGlobPath() << "</StatsGlobPath>" << endl;
	os << setw(4) << "<StatsOpPath>" << bestAlnsParams->getStatsOpPath() << "</StatsOpPath>" << endl;
	os << setw(4) << "<ForbiddenOperators/>" << endl;
	if(bestAlnsParams->getPerformLocalSearch())
	{
		os << setw(4) << "<PerformLocalSearch>true</PerformLocalSearch>" << endl;
	}
	else
	{
		os << setw(4) << "<PerformLocalSearch>true</PerformLocalSearch>" << endl;
	}
	os << setw(4) << "<ForbiddenLSOperators/>" << endl;
	os << setw(2) << "</ALNS>" << endl;
	os << setw(2) << "<CoolingSchedule>" << endl;
	switch(bestAmParams->kind)
	{
	case CoolingSchedule_Parameters::Linear_it:
		os << setw(4) << "<Kind>Linear_it</Kind>" << endl;
		break;
	case CoolingSchedule_Parameters::Linear_time:
		os << setw(4) << "<Kind>Linear_time</Kind>" << endl;
		break;
	case CoolingSchedule_Parameters::Linear_mix:
		os << setw(4) << "<Kind>Linear_mix</Kind>" << endl;
		break;
	case CoolingSchedule_Parameters::Exponential_it:
		os << setw(4) << "<Kind>Exponential_it</Kind>" << endl;
		break;
	case CoolingSchedule_Parameters::Exponential_time:
		os << setw(4) << "<Kind>Exponential_time</Kind>" << endl;
		break;
	case CoolingSchedule_Parameters::Exponential_mix:
		os << setw(4) << "<Kind>Exponential_mix</Kind>" << endl;
		break;
	}
	os << setw(4) << "<ExpPercentageKept>" << bestAmParams->expPercentageKept << "</ExpPercentageKept>" << endl;
	os << setw(4) << "<SetupPercentage>" << bestAmParams->setupPercentage << "</SetupPercentage>" << endl;
	os << setw(4) << "<NbThresholds>" << bestAmParams->nbThresholds << "</NbThresholds>" << endl;
	os << setw(2) << "</CoolingSchedule>" << endl;
	os << "</Parameters>" << endl;
}

void ParameterTuning::optimizeParameters(std::string outputPath)
{
	time_t start = time(0);
	bestScore = evaluateConfiguration(*alnsParams,*amParams);
	time_t currentTime = time(0);
	while(difftime(currentTime, start)<maximumRuntime)
	{
		ALNS_Parameters_optimizable* p1 = new ALNS_Parameters_optimizable(*bestAlnsParams);
		CoolingSchedule_Parameters_Optimizable* p2 = new CoolingSchedule_Parameters_Optimizable(*bestAmParams);
		modifyParameters(p1,p2);
		double currScore = evaluateConfiguration(*p1,*p2);
		if(currScore < bestScore)
		{
			delete bestAlnsParams;
			delete bestAmParams;
			bestAlnsParams = new ALNS_Parameters_optimizable(*p1);
			bestAmParams = new CoolingSchedule_Parameters_Optimizable(*p2);
			bestScore = currScore;
		}
		delete p1;
		delete p2;
		currentTime = time(0);
	}
	writeBestParameters(outputPath);
}
