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

#ifndef AOPERATOR_H_
#define AOPERATOR_H_

#include <iostream>

/*!
 * \class AOperator.
 * \brief This abstract class represent an operator, Destroy and Repair Operators inherit
 * from this class.
 *
 *
 */

class AOperator
{
private:
	//! Total number of calls during the process.
	size_t totalNumberOfCalls;

	//! Number of calls since the last evaluation.
	size_t nbCallsSinceLastEval;

	//! score of the operator.
	double score;

	//! weight of the operator.
	double weight;

	//! designation of the operator.
	std::string operatorName;

protected:
	//! Indicate if the operator is used in noise mode or not.
	bool noise;
public:

	//! Constructor.
	AOperator(std::string name){
		operatorName = name;
		init();
	}

	//! Destructor.
	virtual ~AOperator(){};

	//! Initialize the values of the numbers of calls.
	void init()
	{
		totalNumberOfCalls = 0;
		nbCallsSinceLastEval = 0;
		score = 0;
		weight = 1;
	}

	//! reset the number of calls since last eval.
	void resetNumberOfCalls()
	{
		nbCallsSinceLastEval = 0;
	}

	//! Simple getter.
	//! \return the total number of calls to the operator since
	//! the beginning of the optimization process.
	size_t getTotalNumberOfCalls(){return totalNumberOfCalls;};

	//! Simple getter.
	//! \return the number of calls to this operator since the last
	//! evaluation.
	size_t getNumberOfCallsSinceLastEvaluation(){return nbCallsSinceLastEval;};

	void increaseNumberOfCalls()
	{
		totalNumberOfCalls++;
		nbCallsSinceLastEval++;
	}

	//! Simple getter.
	double getScore() const
    {
        return score;
    }

	//! Simple getter.
    double getWeight() const
    {
        return weight;
    }

    //! resetter.
    void resetScore()
    {
        this->score = 0;
    }

    //! Simple setter.
	void setScore(double s)
	{
		this->score = s;
	}

    //! Simple setter.
    void setWeight(double weight)
    {
        this->weight = weight;
    }

    //! Simple getter.
    std::string getName(){return operatorName;};

    //! Set noise to true.
    void setNoise(){noise=true;};

    //! Set noise to false.
    void unsetNoise(){noise=false;};



};


#endif /* AOPERATOR_H_ */
