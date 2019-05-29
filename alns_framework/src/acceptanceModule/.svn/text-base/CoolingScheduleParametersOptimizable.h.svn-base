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
 *  CoolingScheduleParametersOptimizable.h
 * Contributor(s):
 *	Renaud Masson
 */

#ifndef COOLINGSCHEDULEPARAMETERSOPTIMIZABLE_H_
#define COOLINGSCHEDULEPARAMETERSOPTIMIZABLE_H_

#import "CoolingSchedule_Parameters.h"
/*! \class CoolingScheduleParametersOptimizable.h CoolingScheduleParametersOptimizable.h.h "/ALNS-framework/trunk/src/acceptanceModule/CoolingScheduleParametersOptimizable.h"
 *  \brief This class represent the range of values that the parameters of the method can take.
 *
 *  Detailed Description.
 */

/*
 *
 */
class CoolingSchedule_Parameters_Optimizable: public CoolingSchedule_Parameters {
public:
	CoolingSchedule_Parameters_Optimizable(ALNS_Parameters& params);

	CoolingSchedule_Parameters_Optimizable(CoolingSchedule_Parameters_Optimizable& p);

	virtual ~CoolingSchedule_Parameters_Optimizable();

	double getLbExpPercentageKept() const {
		return lbExpPercentageKept;
	}

	void setLbExpPercentageKept(double lbExpPercentageKept) {
		this->lbExpPercentageKept = lbExpPercentageKept;
	}

	double getLbSetupPercentage() const {
		return lbSetupPercentage;
	}

	void setLbSetupPercentage(double lbSetupPercentage) {
		this->lbSetupPercentage = lbSetupPercentage;
	}

	double getUbExpPercentageKept() const {
		return ubExpPercentageKept;
	}

	void setUbExpPercentageKept(double ubExpPercentageKept) {
		this->ubExpPercentageKept = ubExpPercentageKept;
	}

	double getUbSetupPercentage() const {
		return ubSetupPercentage;
	}

	void setUbSetupPercentage(double ubSetupPercentage) {
		this->ubSetupPercentage = ubSetupPercentage;
	}

	//! \return the hash of the parameter set.
	std::string getHash();

	//! \return the number of tunable parameters from this set.
	int getNbTunableParameters(){return 2;};

	//! Modify randomly one of the parameters while staying
	//! within the parameter bounds.
	void modifyOneParameter();

	//! Modify the value of the parameter expPercentageKept.
	void modifyExpPercentageKept();

	//! modify the value of the parameter setupPercentage.
	void modifySetupPercentage();

	//! Load the parameters of the file.
	//! \param path the path to the parameters.
	void loadAllXMLParameters(std::string path);

protected:

	//! Lower bound for parameter expPercentageKept
	double lbExpPercentageKept;

	//! Upper bound for parameter expPercentageKept
	double ubExpPercentageKept;

	//! Lower bound for parameter setupPercentage
	double lbSetupPercentage;

	//! Upper bound for parameter setupPercentage
	double ubSetupPercentage;
};

#endif /* COOLINGSCHEDULEPARAMETERSOPTIMIZABLE_H_ */
