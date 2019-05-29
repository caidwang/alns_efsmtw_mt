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
#include <iostream>
#include <stdlib.h>
#include "CoolingScheduleFactory.h"
#include "CoolingSchedule_Parameters.h"
#include "ICoolingSchedule.h"
#include "LinearCoolingSchedule.h"
#include "TimeLinearCoolingSchedule.h"
#include "MixLinearCoolingSchedule.h"
#include "ExponentialCoolingSchedule.h"
#include "../alns/ISolution.h"


ICoolingSchedule* CoolingScheduleFactory::makeCoolingSchedule(ISolution& sol,
															 CoolingSchedule_Parameters& param)
{
	// Depending of the value of the field kind
	// of the cooling schedule parameters
	// a corresponding cooling schedule is build.
	if(param.kind == CoolingSchedule_Parameters::Linear_it)
	{
		std::cout << "LI" << std::endl;
		return dynamic_cast<ICoolingSchedule*>(new LinearCoolingSchedule(sol,param,param.maxIt));
	}
	else if(param.kind == CoolingSchedule_Parameters::Linear_time)
	{
		std::cout << "LT" << std::endl;
		return dynamic_cast<ICoolingSchedule*>(new TimeLinearCoolingSchedule(sol,param));
	}
	else if(param.kind == CoolingSchedule_Parameters::Linear_mix)
	{
		std::cout << "LM" << std::endl;
		return dynamic_cast<ICoolingSchedule*>(new MixLinearCoolingSchedule(sol,param));
	}
	else if(param.kind == CoolingSchedule_Parameters::Exponential_mix)
	{
		std::cout << "EXP" << std::endl;
		return dynamic_cast<ICoolingSchedule*>(new ExponentialCoolingSchedule(sol,param));
	}
	else
	{
		std::cerr << "This cooling schedule is not available yet" << std::endl;
		exit(EXIT_FAILURE);
	}
}


