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

#ifndef ALNS_INC_H_
#define ALNS_INC_H_
#include "acceptanceModule/CoolingScheduleFactory.h"
#include "acceptanceModule/CoolingSchedule_Parameters.h"
#include "acceptanceModule/CoolingScheduleParametersOptimizable.h"
#include "acceptanceModule/ExponentialCoolingSchedule.h"
#include "acceptanceModule/IAcceptanceModule.h"
#include "acceptanceModule/ICoolingSchedule.h"
#include "acceptanceModule/LinearCoolingSchedule.h"
#include "acceptanceModule/MixLinearCoolingSchedule.h"
#include "acceptanceModule/SimulatedAnnealing.h"
#include "acceptanceModule/TimeLinearCoolingSchedule.h"
#include "acceptanceModule/DummyAcceptanceModule.h"
#include "alns/ADestroyOperator.h"
#include "alns/ALNS_Iteration_Status.h"
#include "alns/ALNS_Parameters.h"
#include "alns/ALNSParametersoptimizable.h"
#include "alns/ALNS.h"
#include "alns/AOperator.h"
#include "alns/ARepairOperator.h"
#include "alns/AOperatorManager.h"
#include "alns/IBestSolutionManager.h"
#include "alns/ISolution.h"
#include "alns/IUpdatable.h"
#include "alns/OperatorManager.h"
#include "alns/SimpleBestSolutionManager.h"
#include "localsearch/ILocalSearch.h"
#include "localsearch/ILocalSearchManager.h"
#include "localsearch/SimpleLocalSearchManager.h"
#include "statistics/Statistics.h"
#include "tuning/ISolverBuilder.h"
#include "tuning/ParameterTuning.h"
#endif /* ALNS_INC_H_ */
