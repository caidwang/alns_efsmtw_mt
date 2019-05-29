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
 *  ISolverBuilder.h
 * Contributor(s):
 *	Renaud Masson
 */

#ifndef ISOLVERBUILDER_H_
#define ISOLVERBUILDER_H_

/*! \class ISolverBuilder.h ISolverBuilder.h.h "/ALNS-framework/trunk/src/tuning/ISolverBuilder.h"
 *  \brief This interface describe a Solver builder.
 *
 *  A solver builder create an ALNS solver "populated" with its .
 */

class ALNS;
class ALNS_Parameters_optimizable;
class CoolingSchedule_Parameters_Optimizable;
class ISolution;

class ISolverBuilder {
public:
	ISolverBuilder(){};
	virtual ~ISolverBuilder(){};

	//! Build an ALNS solver.
	//! \param ap the ALNS parameters.
	//! \param cp the cooling schedule parameters.
	//! \param s the ISolution of the instance to be solved.
	//! \return an ALNS solver ready to solve.
	virtual ALNS* buildSolver(ALNS_Parameters_optimizable* ap, CoolingSchedule_Parameters_Optimizable*, ISolution* s) = 0;
};

#endif /* ISOLVERBUILDER_H_ */
