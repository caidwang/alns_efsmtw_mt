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

#ifndef OPERATORMANAGER_H_
#define OPERATORMANAGER_H_

#include <vector>
#include "AOperatorManager.h"


class ARepairOperator;
class ADestroyOperator;
class ALNS_Parameters;
class ALNS_Iteration_Status;
class AOperator;

/*!
 * \class OperatorManager.
 * \brief A simple implementation of an operator manager, that does not allow
 * to simply couple the destroy and repair operators.
 */


class OperatorManager: public AOperatorManager {
private:
	//! The set of repair operators.
	std::vector<AOperator*> repairOperators;

	//! The set of destroy operators.
	std::vector<AOperator*> destroyOperators;

	//! The sum of the weights of the repair operators.
	double sumWeightsRepair;

	//! The sum of the weights of the destroy operators.
	double sumWeightsDestroy;

	//! The paramaters to be used by the ALNS.
	ALNS_Parameters* parameters;

	//! Indicate whether or not the next operators to be return
	//! should be noised or not.
	bool noise;



	//! Use a roulette wheel to select an operator in a vector of operators.
	//! \return the selected operator.
	AOperator& selectOperator(std::vector<AOperator*>& vecOp, double sumW);

	//! Recompute the weight of an operator.
	void recomputeWeight(AOperator& op, double& sumW);
public:
	//! Constructor
	//! \param param the parameters to be used.
	OperatorManager(ALNS_Parameters& param);

	//! Destructor.
	virtual ~OperatorManager();

	//! This function recompute the weights of every operator managed by this
	//! manager.
	void recomputeWeights();

	//! This method selects a destroy operator.
	//! \return a destroy operator.
	ADestroyOperator& selectDestroyOperator();

	//! This method selects a repair operator.
	//! \return a repair operator.
	ARepairOperator& selectRepairOperator();

	//! This method adds a repair operator to the list
	//! of repair operator managed by this manager.
	//! \param repairOperator the repair operator to be added.
	void addRepairOperator(ARepairOperator& repairOperator);

	//! This method adds a destroy operator to the list
	//! of destroy operator managed by this manager.
	//! \param destroyOperator the destroy operator to be added.
	void addDestroyOperator(ADestroyOperator& destroyOperator);

	//! This method run some sanity checks to ensure that it is possible
	//! to "safely" use this manager within the ALNS.
	void sanityChecks();

	//! Update the scores of the operators.
	virtual void updateScores(ADestroyOperator& des, ARepairOperator& rep, ALNS_Iteration_Status& status);

	//! Indicate that the optimization process starts.
	virtual void startSignal();

};

#endif /* OPERATORMANAGER_H_ */
