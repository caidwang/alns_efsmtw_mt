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
#ifndef ALNS_ITERATION_STATUS_H_
#define ALNS_ITERATION_STATUS_H_

#include <string>

/*!
 * \class ALNS_Iteration_Status.
 * \brief This class represent the output of an iteration of the ALNS.
 *
 */

class ALNS_Iteration_Status
{
public:

	//! An enumeration representing a boolean with an unknown state.
	enum State
	{ TRUE,
	  FALSE,
	  UNKNOWN
	};

	//! Constructor.
	ALNS_Iteration_Status()
	{
		// The constructor initialize the variables with default values.
		iterationId = 0;
		nbIterationWithoutImprovementSinceLastReload = 0;
		nbIterationWithoutImprovement = 0;
		nbIterationWithoutImprovementCurrent = 0;
		nbIterationWithoutTransition = 0;
		acceptedAsCurrentSolution = UNKNOWN;
		alreadyKnownSolution = UNKNOWN;
		improveCurrentSolution = UNKNOWN;
		newBestSolution = UNKNOWN;
		localSearchUsed = UNKNOWN;
		improveByLocalSearch = UNKNOWN;
		alreadyDestroyed = UNKNOWN;
		alreadyRepaired = UNKNOWN;
	}

	//! Reinitialize some values.
	void partialReinit()
	{
		setAcceptedAsCurrentSolution(UNKNOWN);
		setAlreadyKnownSolution(UNKNOWN);
		setImproveCurrentSolution(UNKNOWN);
		setNewBestSolution(UNKNOWN);
		setLocalSearchUsed(UNKNOWN);
		setImproveByLocalSearch(UNKNOWN);
		setAlreadyDestroyed(UNKNOWN);
		setAlreadyRepaired(UNKNOWN);
	}

	//! Simple getter.
    State getAcceptedAsCurrentSolution() const
    {
        return acceptedAsCurrentSolution;
    }

    //! Simple getter.
    State getAlreadyKnownSolution() const
    {
        return alreadyKnownSolution;
    }

    //! Simple getter.
    State getImproveCurrentSolution() const
    {
        return improveCurrentSolution;
    }

    //! Simple getter.
    size_t getIterationId() const
    {
        return iterationId;
    }



    //! Simple getter.
    size_t getNbIterationWithoutImprovement() const
    {
        return nbIterationWithoutImprovement;
    }

    //! Simple getter.
    size_t getNbIterationWithoutImprovementCurrent() const
    {
        return nbIterationWithoutImprovementCurrent;
    }

    //! Simple getter.
    size_t getNbIterationWithoutTransition() const
    {
        return nbIterationWithoutTransition;
    }

    //! Simple getter.
    State getNewBestSolution() const
    {
        return newBestSolution;
    }

    //! Simple setter.
    void setAcceptedAsCurrentSolution(State acceptedAsCurrentSolution)
    {
        this->acceptedAsCurrentSolution = acceptedAsCurrentSolution;
    }

    //! Simple setter.
    void setAlreadyKnownSolution(State alreadyKnownSolution)
    {
        this->alreadyKnownSolution = alreadyKnownSolution;
    }

    //! Simple setter.
    void setImproveCurrentSolution(State improveCurrentSolution)
    {
        this->improveCurrentSolution = improveCurrentSolution;
    }

    //! Simple setter.
    void setIterationId(size_t iterationId)
    {
        this->iterationId = iterationId;
    }


    //! Simple setter.
    void setNbIterationWithoutImprovement(size_t nbIterationWithoutImprovement)
    {
        this->nbIterationWithoutImprovement = nbIterationWithoutImprovement;
    }

    //! Simple setter.
    void setNbIterationWithoutImprovementCurrent(size_t nbIterationWithoutImprovementCurrent)
    {
        this->nbIterationWithoutImprovementCurrent = nbIterationWithoutImprovementCurrent;
    }

    //! Simple setter.
    void setNbIterationWithoutTransition(size_t nbIterationWithoutTransition)
    {
        this->nbIterationWithoutTransition = nbIterationWithoutTransition;
    }

    //! Simple setter.
    void setNewBestSolution(State newBestSolution)
    {
        this->newBestSolution = newBestSolution;
    }

    //! Destructor.
    ~ALNS_Iteration_Status(){};

    //! Simple getter.
    State getImproveByLocalSearch() const
	{
		return improveByLocalSearch;
	}

    //! Simple getter.
	State getLocalSearchUsed() const
	{
		return localSearchUsed;
	}

	//! Simple setter.
	void setImproveByLocalSearch(State improveByLocalSearch)
	{
		this->improveByLocalSearch = improveByLocalSearch;
	}

	//! Simple setter.
	void setLocalSearchUsed(State localSearchUsed)
	{
		this->localSearchUsed = localSearchUsed;
	}

	size_t getNbIterationWithoutImprovementSinceLastReload() const
	{
		return nbIterationWithoutImprovementSinceLastReload;
	}

	void setNbIterationWithoutImprovementSinceLastReload(size_t nb)
	{
		nbIterationWithoutImprovementSinceLastReload = nb;
	}

	State getAlreadyDestroyed() const {
		return alreadyDestroyed;
	}

	void setAlreadyDestroyed(State alreadyDestroyed) {
		this->alreadyDestroyed = alreadyDestroyed;
	}

	State getAlreadyRepaired() const {
		return alreadyRepaired;
	}

	void setAlreadyRepaired(State alreadyRepaired) {
		this->alreadyRepaired = alreadyRepaired;
	}

private:

	//! Id of the iteration corresponding to this status.
	size_t iterationId;

	//! Number of iteration since the last improvement of the BKS
	size_t nbIterationWithoutImprovement;

	//! Number of iteration since the last improvement of the BKS
	//! or the last reload of the best known solution.
	size_t nbIterationWithoutImprovementSinceLastReload;

	//! Number of iterations since the last improvement of the current
	//! solution.
	size_t nbIterationWithoutImprovementCurrent;

	//! Number of iterations without transition.
	size_t nbIterationWithoutTransition;

	//! Indicate if a new best solution has been obtained.
	State newBestSolution;

	//! Indicate if the new solution has been accepted as the
	//! current solution.
	State acceptedAsCurrentSolution;

	//! Indicate if the new solution is already known.
	State alreadyKnownSolution;

	//! Indicate if the new solution improve the current solution.
	State improveCurrentSolution;

	//! Indicate if a local search operator has been used.
	State localSearchUsed;

	//! Indicate if solution has been improved by local search.
	State improveByLocalSearch;

	//! Indicate if the solution has already been repaired.
	State alreadyRepaired;

	//! Indicate if the new solution has already been destroyed.
	State alreadyDestroyed;

};

#endif /* ALNS_ITERATION_STATUS_H_ */
