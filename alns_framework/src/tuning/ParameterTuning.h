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
 *  ParameterTuning.h
 * Contributor(s):
 *	Renaud Masson
 */

#ifndef PARAMETERTUNING_H_
#define PARAMETERTUNING_H_

/*! \class ParameterTuning.h ParameterTuning.h.h "/ALNS-framework/trunk/src/tuning/ParameterTuning.h"
 *  \brief This class is used to auto-tune the parameters of the ALNS for a given set of problems.
 */

#include <iostream>
#include <map>
#include <vector>

class ALNS_Parameters_optimizable;
class CoolingSchedule_Parameters_Optimizable;
class ISolution;
class ISolverBuilder;

class ParameterTuning {
public:
	//! Constructor.
	//! \param alnsParams initial set of parameters for the ALNS.
	//! \param amParams initial set of parameters for the Acceptance criterion.
	//! \param trainingSet the problems on which the parameters are optimized.
	//! \param runTime the maximum run time used optimizing the parameters.
	//! \param nbPerInstance the number of execution on each instance to evaluate a parameter set.
	//! \param nbThreads the number of available threads on the machine on which the parameters
	//! optimization is done.
	//! \param solverFactory a class building ALNS solver for the problem at hand.
	ParameterTuning(ALNS_Parameters_optimizable* alnsParams,
				      CoolingSchedule_Parameters_Optimizable* amParams,
				      std::vector<ISolution*> trainingSet,
				      long long runTime,
				      int nbPerInstance,
				      int nbThreads,
				      ISolverBuilder* solverFactory);
	virtual ~ParameterTuning();

	//! Perform one run of the algorithm with the current parameters on one instance of the problem.
	//! \param ap the ALNS parameters.
	//! \param cp the cooling schedule parameters.
	//! \param s the instance to solve.
	//! \return the cost of the optimized instance.
	void runInstance(ALNS_Parameters_optimizable* ap, CoolingSchedule_Parameters_Optimizable* cp, ISolution* s);


	//! Optimize the parameters and write them in an xml files.
	//! \param outputPath the path in which the parameters should be written.
	void optimizeParameters(std::string outputPath);


protected:
	//! The initial parameter set.
	ALNS_Parameters_optimizable* alnsParams;
	//! The initial cooling schedule parameter set.
	CoolingSchedule_Parameters_Optimizable* amParams;

	//! The best parameter set.
	ALNS_Parameters_optimizable* bestAlnsParams;
	//! The best cooling schedule parameter set.
	CoolingSchedule_Parameters_Optimizable* bestAmParams;

	//! The solver builder to be used to construct the ALNS solvers.
	ISolverBuilder* solverBuilder;

	//! For each parameter set that is evaluated we compute a hash
	//! and store the associated score.
	std::map<std::string, double> scores;

	//! The score of the best configuration.
	double bestScore;

	//! The problem set on which we train the parameters.
	std::vector<ISolution*> problemSet;

	//! The maximum run time after which the tuning tool should stop
	//! and return the best parameter set identified.
	long long maximumRuntime;

	//! The number of run done for each instance.
	int nbRunPerInstance;

	//! The maximum number of threads that can be used in parallel
	//! by the method.
	int maximumNbThreads;

	//! The number of threads currently available.
	int availableThreads;

	//! This data structure hold the scores of the instances being currently evaluated.
	std::vector<double> currentScores;

	//! Build a hash for a parameter set.
	//! \param ap the ALNS parameters.
	//! \param cp the Cooling parameters.
	//! \return the hash of the parameter set.
	std::string buildHash(ALNS_Parameters_optimizable& ap, CoolingSchedule_Parameters_Optimizable& cp);


	//! Modify one of the parameters.
	void modifyParameters(ALNS_Parameters_optimizable* ap, CoolingSchedule_Parameters_Optimizable* cp);

	//! Compute the score associated to the two parameters sets.
	//! \param ap the ALNS parameters.
	//! \param cp the Cooling schedule parameters.
	//! \return the score associated to the parameters set.
	double evaluateConfiguration(ALNS_Parameters_optimizable& ap, CoolingSchedule_Parameters_Optimizable& cp);

	//! Write the best parameters to an XML file.
	//! \param path the path of the file in which the parameters will be written.
	void writeBestParameters(std::string path);


};

#endif /* PARAMETERTUNING_H_ */
