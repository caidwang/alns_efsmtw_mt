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

#ifndef AREPAIROPERATOR_H_
#define AREPAIROPERATOR_H_
#include <iostream>
#include "AOperator.h"

class ISolution;

/*!
 * \class ARepairOperator.
 * \brief This abstract class represent a Repair Operator, all repair operator implementations
 * should inherit from this class.
 *
 *
 */


class ARepairOperator : public AOperator {

public:
	ARepairOperator(std::string s) : AOperator(s)
	{
	}

	virtual ~ARepairOperator(){};

	virtual void repairSolution(ISolution& sol)=0;
};

#endif /* AREPAIROPERATOR_H_ */
