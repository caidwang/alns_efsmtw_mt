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
 *  ALNSParametersoptimizable.h
 * Contributor(s):
 *	Renaud Masson
 */

#ifndef ALNSPARAMETERSOPTIMIZABLE_H_
#define ALNSPARAMETERSOPTIMIZABLE_H_

#include "ALNS_Parameters.h"

/*! \class ALNSParametersoptimizable.h ALNSParametersoptimizable.h.h "/ALNS-framework/trunk/src/alns/ALNSParametersoptimizable.h"
 *  \brief This class describe the range of values that the parameters of the method can take.
 */

class ALNS_Parameters_optimizable: public ALNS_Parameters {
public:
	ALNS_Parameters_optimizable();

	//! Copy constructor.
	ALNS_Parameters_optimizable(ALNS_Parameters_optimizable& p);

	//! \return a hash representing the current parameter set.
	std::string getHash();

	virtual ~ALNS_Parameters_optimizable();

	double getLbMaximumWeight() const {
		return lbMaximumWeight;
	}

	void setLbMaximumWeight(double lbMaximumWeight) {
		this->lbMaximumWeight = lbMaximumWeight;
	}

	double getLbMinimumWeight() const {
		return lbMinimumWeight;
	}

	void setLbMinimumWeight(double lbMinimumWeight) {
		this->lbMinimumWeight = lbMinimumWeight;
	}

	size_t getLbNbItBeforReinit() const {
		return lbNbItBeforReinit;
	}

	void setLbNbItBeforReinit(size_t lbNbItBeforReinit) {
		this->lbNbItBeforReinit = lbNbItBeforReinit;
	}

	double getLbProbabilityOfNoise() const {
		return lbProbabilityOfNoise;
	}

	void setLbProbabilityOfNoise(double lbProbabilityOfNoise) {
		this->lbProbabilityOfNoise = lbProbabilityOfNoise;
	}

	size_t getLbReloadFrequency() const {
		return lbReloadFrequency;
	}

	void setLbReloadFrequency(size_t lbReloadFrequency) {
		this->lbReloadFrequency = lbReloadFrequency;
	}

	double getLbRho() const {
		return lbRho;
	}

	void setLbRho(double lbRho) {
		this->lbRho = lbRho;
	}

	int getLbSigma1() const {
		return lbSigma1;
	}

	void setLbSigma1(int lbSigma1) {
		this->lbSigma1 = lbSigma1;
	}

	int getLbSigma2() const {
		return lbSigma2;
	}

	void setLbSigma2(int lbSigma2) {
		this->lbSigma2 = lbSigma2;
	}

	int getLbSigma3() const {
		return lbSigma3;
	}

	void setLbSigma3(int lbSigma3) {
		this->lbSigma3 = lbSigma3;
	}

	size_t getLbTimeSegmentsIt() const {
		return lbTimeSegmentsIt;
	}

	void setLbTimeSegmentsIt(size_t lbTimeSegmentsIt) {
		this->lbTimeSegmentsIt = lbTimeSegmentsIt;
	}

	double getUbMaximumWeight() const {
		return ubMaximumWeight;
	}

	void setUbMaximumWeight(double ubMaximumWeight) {
		this->ubMaximumWeight = ubMaximumWeight;
	}

	double getUbMinimumWeight() const {
		return ubMinimumWeight;
	}

	void setUbMinimumWeight(double ubMinimumWeight) {
		this->ubMinimumWeight = ubMinimumWeight;
	}

	size_t getUbNbItBeforReinit() const {
		return ubNbItBeforReinit;
	}

	void setUbNbItBeforReinit(size_t ubNbItBeforReinit) {
		this->ubNbItBeforReinit = ubNbItBeforReinit;
	}

	double getUbProbabilityOfNoise() const {
		return ubProbabilityOfNoise;
	}

	void setUbProbabilityOfNoise(double ubProbabilityOfNoise) {
		this->ubProbabilityOfNoise = ubProbabilityOfNoise;
	}

	size_t getUbReloadFrequency() const {
		return ubReloadFrequency;
	}

	void setUbReloadFrequency(size_t ubReloadFrequency) {
		this->ubReloadFrequency = ubReloadFrequency;
	}

	double getUbRho() const {
		return ubRho;
	}

	void setUbRho(double ubRho) {
		this->ubRho = ubRho;
	}

	int getUbSigma1() const {
		return ubSigma1;
	}

	void setUbSigma1(int ubSigma1) {
		this->ubSigma1 = ubSigma1;
	}

	int getUbSigma2() const {
		return ubSigma2;
	}

	void setUbSigma2(int ubSigma2) {
		this->ubSigma2 = ubSigma2;
	}

	int getUbSigma3() const {
		return ubSigma3;
	}

	void setUbSigma3(int ubSigma3) {
		this->ubSigma3 = ubSigma3;
	}

	size_t getUbTimeSegmentsIt() const {
		return ubTimeSegmentsIt;
	}

	void setUbTimeSegmentsIt(size_t ubTimeSegmentsIt) {
		this->ubTimeSegmentsIt = ubTimeSegmentsIt;
	}

	int getLbMaxDestroyPerc() const {
		return lbMaxDestroyPerc;
	}

	void setLbMaxDestroyPerc(int lbMaxDestroyPerc) {
		this->lbMaxDestroyPerc = lbMaxDestroyPerc;
	}

	int getLbMinDestroyPerc() const {
		return lbMinDestroyPerc;
	}

	void setLbMinDestroyPerc(int lbMinDestroyPerc) {
		this->lbMinDestroyPerc = lbMinDestroyPerc;
	}

	int getUbMaxDestroyPerc() const {
		return ubMaxDestroyPerc;
	}

	void setUbMaxDestroyPerc(int ubMaxDestroyPerc) {
		this->ubMaxDestroyPerc = ubMaxDestroyPerc;
	}

	int getUbMinDestroyPerc() const {
		return ubMinDestroyPerc;
	}

	void setUbMinDestroyPerc(int ubMinDestroyPerc) {
		this->ubMinDestroyPerc = ubMinDestroyPerc;
	}

	//! \return the number of tunable parameters from this set.
	int getNbTunableParameters(){return 12;};

	//! Modify randomly one of the parameters while staying
	//! within the parameter bounds.
	void modifyOneParameter();

	//! Load the parameters.
	//! \param path the path to the file in which the parameters
	//! are stored.
	void loadAllXMLParameters(std::string path);


protected:

	//! Lower bound for parameter timeSegmentsIt.
	size_t lbTimeSegmentsIt;

	//! Upper bound for parameter timeSegmentsIt.
	size_t ubTimeSegmentsIt;

	//! Lower bound for parameter nbItBeforReinit.
	size_t lbNbItBeforReinit;

	//! Upper bound for parameter nbItBeforReinit.
	size_t ubNbItBeforReinit;

	//! Lower bound for parameter sigma1
	int lbSigma1;

	//! Upper bound for parameter sigma1
	int ubSigma1;

	//! Lower bound for parameter sigma2
	int lbSigma2;

	//! Upper bound for parameter sigma2
	int ubSigma2;

	//! Lower bound for parameter sigma3
	int lbSigma3;

	//! Upper bound for parameter sigma3
	int ubSigma3;

	//! Lower bound for parameter rho
	double lbRho;

	//! Upper bound for parameter rho
	double ubRho;

	//! Lower bound for parameter minimumWeight
	double lbMinimumWeight;

	//! Upper bound for parameter minimumWeight
	double ubMinimumWeight;

	//! Lower bound for parameter maximumWeight
	double lbMaximumWeight;

	//! Upper bound for parameter maximumWeight
	double ubMaximumWeight;

	//! Lower bound for parameter probabilityOfNoise
	double lbProbabilityOfNoise;

	//! Upper bound for parameter probabilityOfNoise
	double ubProbabilityOfNoise;

	//! Lower bound for parameter minDestroyPerc.
	int lbMinDestroyPerc;

	//! Upper bound for parameter minDestroyPerc.
	int ubMinDestroyPerc;

	//! Lower bound for parameter maxDestroyPerc.
	int lbMaxDestroyPerc;

	//! Upper bound for parameter maxDestroyPerc.
	int ubMaxDestroyPerc;

	//! Lower bound for parameter reloadFrequency
	size_t lbReloadFrequency;

	//! Upper bound for parameter reloadFrequency
	size_t ubReloadFrequency;

	void modifyTimeSegmentsIt();

	void modifyNbItBeforReinit();

	void modifySigma1();

	void modifySigma2();

	void modifySigma3();

	void modifyRho();

	void modifyMinimumWeight();

	void modifyMaximumWeight();

	void modifyProbabilityOfNoise();

	void modifyMinDestroy();

	void modifyMaxDestroy();

	void modifyReloadFrequency();
};

#endif /* ALNSPARAMETERSOPTIMIZABLE_H_ */
