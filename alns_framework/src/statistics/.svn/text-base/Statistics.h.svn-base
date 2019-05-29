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
#ifndef STATISTICS_H_
#define STATISTICS_H_

#include <vector>
#include <time.h>

class Statistics {
public:
	//! Constructor.
	Statistics(){};

	//! Destructor.
	virtual ~Statistics();

	//! This method adds an entry to the data
	void addEntry(double timeStamp,
				  size_t iteration,
				  std::string destroyName,
				  std::string recreateName,
				  double newCost,
				  double currentCost,
				  double bestCost,
				  int cumKS);

	void addOperatorEntry(std::vector<double>* weight,
						  std::vector<size_t>* calls);

	void addOperatorsNames(std::vector<std::string>* names){operatorNames = names;};




	//! This method generate the file containing the datas.
	void generateStatsFile(std::string path, std::string pathOp);

	void setStart(){start = clock();};

private:
	std::vector<double> timeStamps;
	std::vector<size_t> iterations;
	std::vector<std::string> destroyNames;
	std::vector<std::string> recreateNames;
	std::vector<double> newCosts;
	std::vector<double> currentCosts;
	std::vector<double> bestCosts;
	std::vector<int> cumulativeKnownSolutions;
	std::vector<std::vector<double>* > weights;
	std::vector<std::vector<size_t>* > nbCalls;
	std::vector<double> timeStampsOperators;
	std::vector<std::string>* operatorNames;
	clock_t start;

};

#endif /* STATISTICS_H_ */
