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

#ifndef ADESTROYOPERATOR_H_
#define ADESTROYOPERATOR_H_

#include "AOperator.h"

class ISolution;

/*!
 * \class ADestroyOperator.
 * \brief This is an abstract class used to represent Destroy Operators.
 *
 * Any destroy operator should inherit from this class and implement the
 * destroySolution function.
 */

class ADestroyOperator : public AOperator {
protected:
	//! The minimum destroy size used.
	size_t minimunDestroy;
	//! The maximum destroy size used.
	size_t maximumDestroy;

public:
	//! Constructor.
	//! \param mini the minimum destroy size.
	//! \param maxi the maximum destroy size.
	//! \param s the name of the destroy operator.
	ADestroyOperator(size_t mini, size_t maxi, std::string s) : AOperator(s)
	{
		minimunDestroy = mini;
		maximumDestroy = maxi;
	}

	//! Destructor.
	virtual ~ADestroyOperator(){};

	//! This function is the one called to destroy a solution.
	//! \param sol the solution to be destroyed.
	virtual void destroySolution(ISolution& sol)=0;
};

#endif /* DESTROYOPERATOR_H_ */
