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

#ifndef IOPERATORMANAGER_H_
#define IOPERATORMANAGER_H_

class ADestroyOperator;
class ARepairOperator;
class ALNS_Iteration_Status;
class Statistics;

/*!
 * \class IOperatorManager.
 * \brief This interface represent the methods that should be implemented by an operator
 * manager.
 *
 *
 */

class AOperatorManager
{
public:
	//! This method selects a destroy operator.
	//! \return a destroy operator.
	virtual ADestroyOperator& selectDestroyOperator()=0;

	//! This method selects a repair operator.
	//! \return a repair operator.
	virtual ARepairOperator& selectRepairOperator()=0;

	virtual void recomputeWeights()=0;

	//! Update the scores of the operators.
	virtual void updateScores(ADestroyOperator& des, ARepairOperator& rep, ALNS_Iteration_Status& status)=0;

	//! Indicate that the optimization process starts.
	virtual void startSignal()=0;

	//! Destroy the operators registered to this operator manager.
	virtual void end()=0;

    //! Simple setter.
	void setStatistics(Statistics* statistics){stats = statistics;};
protected:
	//! A pointer to the instance of the statistics module.
	Statistics* stats;
};

#endif /* IOPERATORMANAGER_H_ */
