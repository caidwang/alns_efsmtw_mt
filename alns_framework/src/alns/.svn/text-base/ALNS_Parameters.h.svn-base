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

#ifndef ALNS_PARAMETERS_H_
#define ALNS_PARAMETERS_H_

#include <assert.h>
#include <cstring>
#include <iostream>
#include <vector>

/*!
 * \class ALNS_Parameters.
 * \brief This class contains parameters to be used by the ALNS.
 *
 */

class ALNS_Parameters
{

public:

	//! Enumeration representing the various kind of stopping criteria.
	//! MAX_IT: the maximum number of iterations.
	//! MAX_RT: the maximum run time.
	//! MAX_IT_NO_IMP: the maximum number of iterations without improvement.
	//! ALL: a mix of the MAX_IT, MAX_RT and MAX_IT_NO_IMP.
	enum StoppingCriteria {
		MAX_IT,
		MAX_RT,
		MAX_IT_NO_IMP,
		ALL
	};

	//! An enumeration listing a set of packaged AcceptanceModule Implementation.
	enum AcceptanceCriterioKind {
		SA
	};


	//! Constructor.
	ALNS_Parameters();

	//! Copy constructor.
	ALNS_Parameters(ALNS_Parameters& p);

	//! Destructor.
	~ALNS_Parameters();

	//! This method perform some sanity checks on the value of the parameters.
	void sanityChecks();

	//! Load the parameters of a file.
	//! \param path the path to the file containing the parameters.
	void loadParameters(std::string path);

	//! Load the parameters of a xml file.
	//! \param path the path to the xml file containing the parameters.
	void loadXMLParameters(std::string path);

    size_t getReloadFrequency() const
    {
        return reloadFrequency;
    }

    void setReloadFrequency(size_t reloadFrequency)
    {
    	assert(!lock);
        this->reloadFrequency = reloadFrequency;
    }

    //! Simple getter.
    std::vector<std::string> getForbidenLsOperators() const
    {
        return forbidenLsOperators;
    }

    void addForbiddenLsOperator(std::string lsOp)
    {
    	forbidenLsOperators.push_back(lsOp);
    }

    //! Simple getter.
    std::vector<std::string> getForbidenOperators() const
    {
        return forbidenOperators;
    }

    void addForbiddenOperator(std::string op)
    {
    	forbidenOperators.push_back(op);
    }

    bool getPerformLocalSearch() const
    {
        return performLocalSearch;
    }

    void setPerformLocalSearch(bool performLocalSearch)
    {
    	assert(!lock);
        this->performLocalSearch = performLocalSearch;
    }

    int getLogFrequency() const
    {
        return logFrequency;
    }



    void setLogFrequency(int logFrequency)
    {
    	assert(!lock);
        this->logFrequency = logFrequency;
    }



    std::string getStatsGlobPath() const
    {
        return statsGlobPath;
    }

    std::string getStatsOpPath() const
    {
        return statsOpPath;
    }

    void setStatsGlobPath(std::string statsGlobPath)
    {
        this->statsGlobPath = statsGlobPath;
    }

    void setStatsOpPath(std::string statsOpPath)
    {
        this->statsOpPath = statsOpPath;
    }

    //! Simple getter.
    AcceptanceCriterioKind getAcKind() const
    {
        return acKind;
    }

    //! Simple getter.
    std::string getAcPath() const
    {
        return acPath;
    }

    //! Simple getter.
    void setAcKind(AcceptanceCriterioKind acKind)
    {
    	assert(!lock);
        this->acKind = acKind;
    }

    //! Simple getter.
    void setAcPath(std::string acPath)
    {
    	assert(!lock);
        this->acPath = acPath;
    }

    //! Simple getter.
    double getProbabilityOfNoise() const
    {
        return probabilityOfNoise;
    }

    //! Simple getter.
    void setProbabilityOfNoise(double probabilityOfNoise)
    {
    	assert(!lock);
        this->probabilityOfNoise = probabilityOfNoise;
    }

    //! Simple getter.
    size_t getMaxNbIterations() const
    {
        return maxNbIterations;
    }

    //! Simple getter.
    size_t getMaxNbIterationsNoImp() const
    {
        return maxNbIterationsNoImp;
    }

    //! Simple getter.
    double getMaxRunningTime() const
    {
        return maxRunningTime;
    }

    //! Simple getter.
    double getMaximumWeight() const
    {
        return maximumWeight;
    }

    //! Simple getter.
    double getMinimumWeight() const
    {
        return minimumWeight;
    }

    //! Simple getter.
    size_t getNbItBeforeReinit() const
    {
        return nbItBeforeReinit;
    }

    //! Simple getter.
    bool getNoise() const
    {
        return noise;
    }

    //! Simple getter.
    double getRho() const
    {
        return rho;
    }

    //! Simple getter.
    int getSigma1() const
    {
        return sigma1;
    }

    //! Simple getter.
    int getSigma2() const
    {
        return sigma2;
    }

    //! Simple getter.
    int getSigma3() const
    {
        return sigma3;
    }

    //! Simple getter.
    StoppingCriteria getStopCrit() const
    {
        return stopCrit;
    }

    //! Simple getter.
    size_t getTimeSegmentsIt() const
    {
        return timeSegmentsIt;
    }

    //! Simple setter.
    void setMaxNbIterations(size_t maxNbIterations)
    {
    	assert(!lock);
        this->maxNbIterations = maxNbIterations;
    }

    //! Simple setter.
    void setMaxNbIterationsNoImp(size_t maxNbIterationsNoImp)
    {
    	assert(!lock);
        this->maxNbIterationsNoImp = maxNbIterationsNoImp;
    }

    //! Simple setter.
    void setMaxRunningTime(double maxRunningTime)
    {
    	assert(!lock);
        this->maxRunningTime = maxRunningTime;
    }

    //! Simple setter.
    void setMaximumWeight(double maximumWeight)
    {
    	assert(!lock);
        this->maximumWeight = maximumWeight;
    }

    //! Simple setter.
    void setMinimumWeight(double minimumWeight)
    {
    	assert(!lock);
        this->minimumWeight = minimumWeight;
    }

    //! Simple setter.
    void setNbItBeforeReinit(size_t nbItBeforeReinit)
    {
    	assert(!lock);
        this->nbItBeforeReinit = nbItBeforeReinit;
    }

    //! Simple setter.
    void setNoise(bool noise)
    {
    	assert(!lock);
        this->noise = noise;
    }

    //! Simple setter.
    void setRho(double rho)
    {
    	assert(!lock);
        this->rho = rho;
    }

    //! Simple setter.
    void setSigma1(int sigma1)
    {
    	assert(!lock);
        this->sigma1 = sigma1;
    }

    //! Simple setter.
    void setSigma2(int sigma2)
    {
    	assert(!lock);
        this->sigma2 = sigma2;
    }

    //! Simple setter.
    void setSigma3(int sigma3)
    {
    	assert(!lock);
        this->sigma3 = sigma3;
    }

    //! Simple setter.
    void setStopCrit(StoppingCriteria stopCrit)
    {
    	assert(!lock);
        this->stopCrit = stopCrit;
    }

    //! Simple setter.
    void setTimeSegmentsIt(size_t timeSegmentsIt)
    {
    	assert(!lock);
        this->timeSegmentsIt = timeSegmentsIt;
    }

    //! Simple setter.
    void setLock()
    {
    	lock = true;
    }

	int getMaxDestroyPerc() const {
		return maxDestroyPerc;
	}

	void setMaxDestroyPerc(int maxDestroyPerc) {
		this->maxDestroyPerc = maxDestroyPerc;
	}

	int getMinDestroyPerc() const {
		return minDestroyPerc;
	}

	void setMinDestroyPerc(int minDestroyPerc) {
		this->minDestroyPerc = minDestroyPerc;
	}

protected:
	//! Maximum number of iterations performed by the ALNS.
	size_t maxNbIterations;

	//! Maximum running time of the ALNS.
	double maxRunningTime;

	//! Maximum number of iterations without any improvement.
	size_t maxNbIterationsNoImp;

	//! Which stopping criterion should be used.
	StoppingCriteria stopCrit;

	//! Indicate if noise should be used.
	bool noise;

	//! Indicate after how many iterations should the scores of
	//! the operators be recomputed.
	size_t timeSegmentsIt;

	//! Indicate the number of iterations that should be performed
	//! before reinitialization of the scores of the operators.
	size_t nbItBeforeReinit;

	//! score adjustment parameter in case the last remove-insert
	//! operation resulted in a new global best solution
	int sigma1;

	//! score adjustment parameter in case that the last remove-insert
	//! operation resulted in a solution that has not been accepted before and
	//! the objective value is better than the objective value of current solution
	int sigma2;

	//! score adjustment parameter in case that the last remove-insert
	//! operation resulted in a solution that has not been accepted before and such
	//! that the score objective value is worse than the one of current solution but
	//! the solution was accepted.
	int sigma3;

	//! reaction factor 0 <= rho <= 1 for the update of the weights of the
	//! operators.
	double rho;

	//! The minimum possible weight for an operator.
	double minimumWeight;

	//! The maximum possible weight for an operator.
	double maximumWeight;

	//! Indicates the probability of using noised operators.
	double probabilityOfNoise;

	//! Kind of acceptance criterion used.
	AcceptanceCriterioKind acKind;

	//! patht to the configuration file of the acceptance criterion.
	std::string acPath;

	//! path to the file where the global stats have to be saved.
	std::string statsGlobPath;

	//! path to the file where the operators stats have to be saved.
	std::string statsOpPath;

	//! Indicate every each iteration logging is done.
	int logFrequency;

	//! A set of forbidden operators.
	std::vector<std::string> forbidenOperators;

	//! A set of forbidden local search operators.
	std::vector<std::string> forbidenLsOperators;

	//! The minimum percentage of the solution destroyed by the destroy operators.
	int minDestroyPerc;

	//! The maximum percentage of the solution destroyed by the destroy operators.
	int maxDestroyPerc;

	//! Indicate after how many iterations without improvement
	//! does the best known solution is reloaded.
	size_t reloadFrequency;

	//! Indicate if local search should be used.
	bool performLocalSearch;

	//! When the optimization process start, the parameters
	//! should not be modified. lock is set to true when the
	//! optimization begin. If the setter of the value
	//! of one parameter is called while lock is true, an
	//! error is raised.
	bool lock;

};


#endif /* ALNS_PARAMETERS_H_ */
