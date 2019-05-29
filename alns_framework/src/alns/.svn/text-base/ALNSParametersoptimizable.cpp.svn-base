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
 *  ALNSParametersoptimizable.cpp
 * Contributor(s):
 *	Renaud Masson
 */

#include <cstdlib>
#include <sstream>
#include "ALNSParametersoptimizable.h"
#include "../lib/tinyxml/tinyxml.h"

using namespace std;

ALNS_Parameters_optimizable::ALNS_Parameters_optimizable() {
	lbTimeSegmentsIt = timeSegmentsIt;
	ubTimeSegmentsIt = timeSegmentsIt;
	lbNbItBeforReinit = nbItBeforeReinit;
	ubNbItBeforReinit = nbItBeforeReinit;
	lbSigma1 = sigma1;
	ubSigma1 = sigma1;
	lbSigma2 = sigma2;
	ubSigma2 = sigma2;
	lbSigma3 = sigma3;
	ubSigma3 = sigma3;
	lbRho = rho;
	ubRho = rho;
	lbMinimumWeight = minimumWeight;
	ubMinimumWeight = minimumWeight;
	lbMaximumWeight = maximumWeight;
	ubMaximumWeight = maximumWeight;
	lbProbabilityOfNoise = probabilityOfNoise;
	ubProbabilityOfNoise = probabilityOfNoise;
	lbReloadFrequency = reloadFrequency;
	ubReloadFrequency = reloadFrequency;
	lbMinDestroyPerc = minDestroyPerc;
	ubMinDestroyPerc = minDestroyPerc;
	lbMaxDestroyPerc = maxDestroyPerc;
	ubMaxDestroyPerc = maxDestroyPerc;

}

ALNS_Parameters_optimizable::ALNS_Parameters_optimizable(
		ALNS_Parameters_optimizable& p): ALNS_Parameters(p)
{
	lbTimeSegmentsIt = p.lbTimeSegmentsIt;
	ubTimeSegmentsIt = p.ubTimeSegmentsIt;
	lbNbItBeforReinit = p.lbNbItBeforReinit;
	ubNbItBeforReinit = p.ubNbItBeforReinit;
	lbSigma1 = p.lbSigma1;
	ubSigma1 = p.ubSigma1;
	lbSigma2 = p.lbSigma2;
	ubSigma2 = p.ubSigma2;
	lbSigma3 = p.lbSigma3;
	ubSigma3 = p.ubSigma3;
	lbRho = p.rho;
	ubRho = p.rho;
	lbMinimumWeight = p.lbMinimumWeight;
	ubMinimumWeight = p.ubMinimumWeight;
	lbMaximumWeight = p.lbMaximumWeight;
	ubMaximumWeight = p.ubMaximumWeight;
	lbProbabilityOfNoise = p.lbProbabilityOfNoise;
	ubProbabilityOfNoise = p.ubProbabilityOfNoise;
	lbReloadFrequency = p.lbReloadFrequency;
	ubReloadFrequency = p.ubReloadFrequency;
	lbMinDestroyPerc = p.lbMinDestroyPerc;
	ubMinDestroyPerc = p.ubMinDestroyPerc;
	lbMaxDestroyPerc = p.lbMaxDestroyPerc;
	ubMaxDestroyPerc = p.ubMaxDestroyPerc;
}

void ALNS_Parameters_optimizable::loadAllXMLParameters(std::string path)
{
	loadXMLParameters(path);
	TiXmlDocument doc(path.c_str());
	bool loadOkay = doc.LoadFile();
	if (loadOkay)
	{
		TiXmlElement* root = doc.FirstChildElement( "Parameters" );
		if ( root )
		{
			TiXmlElement* paramALNS = root->FirstChildElement( "ALNS" );
			if(paramALNS)
			{
				TiXmlElement* lbTimeSeg = paramALNS->FirstChildElement( "LbTimeSegmentIt" );
				if(lbTimeSeg)
				{
					stringstream str;
					str << lbTimeSeg->GetText();
					str >> this->lbTimeSegmentsIt;
				}
				TiXmlElement* ubTimeSeg = paramALNS->FirstChildElement( "UbTimeSegmentIt" );
				if(ubTimeSeg)
				{
					stringstream str;
					str << ubTimeSeg->GetText();
					str >> this->ubTimeSegmentsIt;
				}
				TiXmlElement* lbNbItBefRe = paramALNS->FirstChildElement( "LbNbItBeforeReinit" );
				if(lbNbItBefRe)
				{
					stringstream str;
					str << lbNbItBefRe->GetText();
					str >> this->lbNbItBeforReinit;
				}
				TiXmlElement* ubNbItBefRe = paramALNS->FirstChildElement( "UbNbItBeforeReinit" );
				if(ubNbItBefRe)
				{
					stringstream str;
					str << ubNbItBefRe->GetText();
					str >> this->ubNbItBeforReinit;
				}
				TiXmlElement* lbSig1 = paramALNS->FirstChildElement( "LbSigma1" );
				if(lbSig1)
				{
					stringstream str;
					str << lbSig1->GetText();
					str >> this->lbSigma1;
				}
				TiXmlElement* ubSig1 = paramALNS->FirstChildElement( "UbSigma1" );
				if(ubSig1)
				{
					stringstream str;
					str << ubSig1->GetText();
					str >> this->ubSigma1;
				}
				TiXmlElement* lbSig2 = paramALNS->FirstChildElement( "LbSigma2" );
				if(lbSig2)
				{
					stringstream str;
					str << lbSig2->GetText();
					str >> this->lbSigma2;
				}
				TiXmlElement* ubSig2 = paramALNS->FirstChildElement( "UbSigma2" );
				if(ubSig2)
				{
					stringstream str;
					str << ubSig2->GetText();
					str >> this->ubSigma2;
				}
				TiXmlElement* lbSig3 = paramALNS->FirstChildElement( "LbSigma3" );
				if(lbSig3)
				{
					stringstream str;
					str << lbSig3->GetText();
					str >> this->lbSigma3;
				}
				TiXmlElement* ubSig3 = paramALNS->FirstChildElement( "UbSigma3" );
				if(ubSig3)
				{
					stringstream str;
					str << ubSig3->GetText();
					str >> this->ubSigma3;
				}
				TiXmlElement* lbRho = paramALNS->FirstChildElement( "LbRho" );
				if(lbRho)
				{
					stringstream str;
					str << lbRho->GetText();
					str >> this->lbRho;
				}
				TiXmlElement* ubRho = paramALNS->FirstChildElement( "UbRho" );
				if(ubRho)
				{
					stringstream str;
					str << ubRho->GetText();
					str >> this->ubRho;
				}
				TiXmlElement* lbMiniW = paramALNS->FirstChildElement( "LbMinimumWeight" );
				if(lbMiniW)
				{
					stringstream str;
					str << lbMiniW->GetText();
					str >> this->lbMinimumWeight;
				}
				TiXmlElement* ubMiniW = paramALNS->FirstChildElement( "UbMinimumWeight" );
				if(ubMiniW)
				{
					stringstream str;
					str << ubMiniW->GetText();
					str >> this->ubMinimumWeight;
				}
				TiXmlElement* lbMaxiW = paramALNS->FirstChildElement( "LbMaximumWeight" );
				if(lbMaxiW)
				{
					stringstream str;
					str << lbMaxiW->GetText();
					str >> this->lbMaximumWeight;
				}
				TiXmlElement* ubMaxiW = paramALNS->FirstChildElement( "UbMaximumWeight" );
				if(ubMaxiW)
				{
					stringstream str;
					str << ubMaxiW->GetText();
					str >> this->ubMaximumWeight;
				}
				TiXmlElement* lbPbNoise = paramALNS->FirstChildElement( "LbProbabilityOfNoise" );
				if(lbPbNoise)
				{
					stringstream str;
					str << lbPbNoise->GetText();
					str >> this->lbProbabilityOfNoise;
				}
				TiXmlElement* ubPbNoise = paramALNS->FirstChildElement( "UbProbabilityOfNoise" );
				if(ubPbNoise)
				{
					stringstream str;
					str << ubPbNoise->GetText();
					str >> this->ubProbabilityOfNoise;
				}
				TiXmlElement* lbRelFreq = paramALNS->FirstChildElement( "LbReloadFrequency" );
				if(lbRelFreq)
				{
					stringstream str;
					str << lbRelFreq->GetText();
					str >> this->lbReloadFrequency;
				}
				TiXmlElement* ubRelFreq = paramALNS->FirstChildElement( "UbReloadFrequency" );
				if(ubRelFreq)
				{
					stringstream str;
					str << ubRelFreq->GetText();
					str >> this->ubReloadFrequency;
				}
				TiXmlElement* lbMinDestP = paramALNS->FirstChildElement( "LbMinDestroyPerc" );
				if(lbMinDestP)
				{
					stringstream str;
					str << lbMinDestP->GetText();
					str >> this->lbMinDestroyPerc;
				}
				TiXmlElement* ubMinDestP = paramALNS->FirstChildElement( "UbMinDestroyPerc" );
				if(ubMinDestP)
				{
					stringstream str;
					str << ubMinDestP->GetText();
					str >> this->ubMinDestroyPerc;
				}
				TiXmlElement* lbMaxDestP = paramALNS->FirstChildElement( "LbMaxDestroyPerc" );
				if(lbMaxDestP)
				{
					stringstream str;
					str << lbMaxDestP->GetText();
					str >> this->lbMaxDestroyPerc;
				}
				TiXmlElement* ubMaxDestP = paramALNS->FirstChildElement( "UbMaxDestroyPerc" );
				if(ubMaxDestP)
				{
					stringstream str;
					str << ubMaxDestP->GetText();
					str >> this->ubMaxDestroyPerc;
				}
			}
		}
	}
}


std::string ALNS_Parameters_optimizable::getHash()
{
	stringstream ss;
	ss << timeSegmentsIt << "-";
	ss << nbItBeforeReinit << "-";
	ss << sigma1 << "-";
	ss << sigma2 << "-";
	ss << sigma3 << "-";
	ss << rho << "-";
	ss << minimumWeight << "-";
	ss << maximumWeight << "-";
	ss << probabilityOfNoise << "-";
	ss << reloadFrequency << "-";
	ss << minDestroyPerc << "-";
	ss << maxDestroyPerc << "-";
	return ss.str();
}

ALNS_Parameters_optimizable::~ALNS_Parameters_optimizable() {

}

void ALNS_Parameters_optimizable::modifyOneParameter()
{
	int selectionParam = rand()%getNbTunableParameters();
	switch(selectionParam)
	{
	case 0:
		modifyMaxDestroy();
		break;
	case 1:
		modifyMaximumWeight();
		break;
	case 2:
		modifyMinDestroy();
		break;
	case 3:
		modifyMinimumWeight();
		break;
	case 4:
		modifyNbItBeforReinit();
		break;
	case 5:
		modifyProbabilityOfNoise();
		break;
	case 6:
		modifyReloadFrequency();
		break;
	case 7:
		modifyRho();
		break;
	case 8:
		modifySigma1();
		break;
	case 9:
		modifySigma2();
		break;
	case 10:
		modifySigma3();
		break;
	case 11:
		modifyTimeSegmentsIt();
		break;
	}
}

void ALNS_Parameters_optimizable::modifyTimeSegmentsIt()
{
	timeSegmentsIt = lbTimeSegmentsIt + rand()%(ubTimeSegmentsIt-lbTimeSegmentsIt);
}

void ALNS_Parameters_optimizable::modifyNbItBeforReinit()
{
	nbItBeforeReinit = lbNbItBeforReinit + rand()%(ubNbItBeforReinit-lbNbItBeforReinit);
}

void ALNS_Parameters_optimizable::modifySigma1()
{
	sigma1 = lbSigma1 + rand()%(ubSigma1-lbSigma1);
}

void ALNS_Parameters_optimizable::modifySigma2()
{
	sigma2 = lbSigma2 + rand()%(ubSigma2-lbSigma2);
}

void ALNS_Parameters_optimizable::modifySigma3()
{
	sigma3 = lbSigma3 + rand()%(ubSigma3-lbSigma3);
}

void ALNS_Parameters_optimizable::modifyRho()
{
	rho = lbRho + (ubRho-lbRho)*(static_cast<double>(rand())/static_cast<double>(RAND_MAX));;
}

void ALNS_Parameters_optimizable::modifyMinimumWeight()
{
	minimumWeight = lbMinimumWeight + (ubMinimumWeight-lbMinimumWeight)*(static_cast<double>(rand())/static_cast<double>(RAND_MAX));;
}

void ALNS_Parameters_optimizable::modifyMaximumWeight()
{
	maximumWeight = lbMaximumWeight + (ubMaximumWeight-lbMaximumWeight)*(static_cast<double>(rand())/static_cast<double>(RAND_MAX));;
}

void ALNS_Parameters_optimizable::modifyProbabilityOfNoise()
{
	probabilityOfNoise = lbProbabilityOfNoise + (ubProbabilityOfNoise-lbProbabilityOfNoise)*(static_cast<double>(rand())/static_cast<double>(RAND_MAX));
}

void ALNS_Parameters_optimizable::modifyMinDestroy()
{
	minDestroyPerc = lbMinDestroyPerc + rand()%(ubMinDestroyPerc-lbMinDestroyPerc);
}

void ALNS_Parameters_optimizable::modifyMaxDestroy()
{
	maxDestroyPerc = lbMaxDestroyPerc + rand()%(ubMaxDestroyPerc-lbMaxDestroyPerc);
}

void ALNS_Parameters_optimizable::modifyReloadFrequency()
{
	reloadFrequency = lbReloadFrequency + rand()%(ubReloadFrequency-lbReloadFrequency);
}
