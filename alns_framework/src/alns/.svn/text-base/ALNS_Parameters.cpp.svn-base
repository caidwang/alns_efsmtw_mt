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

#include "ALNS_Parameters.h"
#include <iostream>
#include <sstream>
#include <fstream>
#include "../lib/tinyxml/tinyxml.h"

using namespace std;

ALNS_Parameters::ALNS_Parameters()
{
	// The variables are initialized with default values.
	maxNbIterations = 20000;

	maxRunningTime = 10;

	maxNbIterationsNoImp = 100000;

	stopCrit = ALL;

	noise = false;

	timeSegmentsIt = 100;

	nbItBeforeReinit = 1000;

	sigma1 = 33;

	sigma2 = 20;

	sigma3 = 15;

	rho = 0.1;

	minimumWeight = 0.1;

	maximumWeight = 5;

	acKind = SA;

	acPath = "";

	logFrequency = 100;

	performLocalSearch = true;

	lock = false;
}

ALNS_Parameters::~ALNS_Parameters()
{

}

void ALNS_Parameters::loadXMLParameters(std::string path)
{
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
				TiXmlElement* maxNbIt = paramALNS->FirstChildElement( "MaxNbIterations" );
				if(maxNbIt)
				{
					stringstream str;
					str << maxNbIt->GetText();
					str >> this->maxNbIterations;
				}
				TiXmlElement* maxRt = paramALNS->FirstChildElement( "MaxRunTime" );
				if(maxRt)
				{
					stringstream str;
					str << maxRt->GetText();
					str >> this->maxRunningTime;
				}
				TiXmlElement* maxNoImp = paramALNS->FirstChildElement( "MaxNbIterationsNoImp" );
				if(maxNoImp)
				{
					stringstream str;
					str << maxNoImp->GetText();
					str >> this->maxNbIterationsNoImp;
				}
				TiXmlElement* stopCriter = paramALNS->FirstChildElement( "StoppingCriterion" );
				if(stopCriter)
				{
					stringstream str;
					string temp;
					str << stopCriter->GetText();
					str >> temp;
					if(temp == "MAX_IT")
					{
						stopCrit = MAX_IT;
					}
					else if(temp == "MAX_RT")
					{
						stopCrit = MAX_RT;
					}
					else if(temp == "MAX_IT_NO_IMP")
					{
						stopCrit = MAX_IT_NO_IMP;
					}
					else if(temp == "ALL")
					{
						stopCrit = ALL;
					}
					else
					{
						assert(false);
					}
				}
				TiXmlElement* noiseP = paramALNS->FirstChildElement( "Noise" );
				if(noiseP)
				{
					stringstream str;
					string temp;
					str << noiseP->GetText();
					str >> temp;
					if(temp=="true")
					{
						noise = true;
					}
					else
					{
						noise = false;
					}
				}
				TiXmlElement* timeSeg = paramALNS->FirstChildElement( "TimeSegmentIt" );
				if(timeSeg)
				{
					stringstream str;
					str << timeSeg->GetText();
					str >> this->timeSegmentsIt;
				}
				TiXmlElement* nbItBefRe = paramALNS->FirstChildElement( "NbItBeforeReinit" );
				if(nbItBefRe)
				{
					stringstream str;
					str << nbItBefRe->GetText();
					str >> this->nbItBeforeReinit;
				}
				TiXmlElement* sig1 = paramALNS->FirstChildElement( "Sigma1" );
				if(sig1)
				{
					stringstream str;
					str << sig1->GetText();
					str >> this->sigma1;
				}
				TiXmlElement* sig2 = paramALNS->FirstChildElement( "Sigma2" );
				if(sig2)
				{
					stringstream str;
					str << sig2->GetText();
					str >> this->sigma1;
				}
				TiXmlElement* sig3 = paramALNS->FirstChildElement( "Sigma3" );
				if(sig3)
				{
					stringstream str;
					str << sig3->GetText();
					str >> this->sigma1;
				}
				TiXmlElement* rho = paramALNS->FirstChildElement( "Rho" );
				if(rho)
				{
					stringstream str;
					str << rho->GetText();
					str >> this->rho;
				}
				TiXmlElement* miniW = paramALNS->FirstChildElement( "MinimumWeight" );
				if(miniW)
				{
					stringstream str;
					str << miniW->GetText();
					str >> this->minimumWeight;
				}
				TiXmlElement* maxiW = paramALNS->FirstChildElement( "MaximumWeight" );
				if(maxiW)
				{
					stringstream str;
					str << maxiW->GetText();
					str >> this->maximumWeight;
				}
				TiXmlElement* pbNoise = paramALNS->FirstChildElement( "ProbabilityOfNoise" );
				if(pbNoise)
				{
					stringstream str;
					str << pbNoise->GetText();
					str >> this->probabilityOfNoise;
				}
				TiXmlElement* acK = paramALNS->FirstChildElement( "AcKind" );
				if(acK)
				{
					stringstream str;
					string temp;
					str << acK->GetText();
					str >> temp;
					if(temp == "SA")
					{
						acKind = SA;
					}
					else
					{
						assert(false);
					}
				}
				TiXmlElement* relFreq = paramALNS->FirstChildElement( "ReloadFrequency" );
				if(relFreq)
				{
					stringstream str;
					str << relFreq->GetText();
					str >> this->reloadFrequency;
				}
				TiXmlElement* sglPath = paramALNS->FirstChildElement( "StatsGlobPath" );
				if(sglPath)
				{
					this->statsGlobPath = sglPath->GetText();
				}
				TiXmlElement* sopPath = paramALNS->FirstChildElement( "StatsOpPath" );
				if(sopPath)
				{
					this->statsOpPath = sopPath->GetText();
				}
				TiXmlElement* logFreq = paramALNS->FirstChildElement( "LogFrequency" );
				if(logFreq)
				{
					stringstream str;
					str << logFreq->GetText();
					str >> this->logFrequency;
				}
				TiXmlElement* minDestP = paramALNS->FirstChildElement( "MinDestroyPerc" );
				if(minDestP)
				{
					stringstream str;
					str << minDestP->GetText();
					str >> this->minDestroyPerc;
				}
				TiXmlElement* maxDestP = paramALNS->FirstChildElement( "MaxDestroyPerc" );
				if(maxDestP)
				{
					stringstream str;
					str << maxDestP->GetText();
					str >> this->maxDestroyPerc;
				}
				TiXmlElement* forbOp = paramALNS->FirstChildElement( "ForbiddenOperators" );
				if(forbOp)
				{
					TiXmlElement* firstForb = forbOp->FirstChildElement();
					while(firstForb)
					{
						addForbiddenOperator(firstForb->GetText());
						firstForb = firstForb->NextSiblingElement();
					}
				}
				TiXmlElement* doLS = paramALNS->FirstChildElement( "PerformLocalSearch" );
				if(doLS)
				{
					stringstream str;
					string temp;
					str << doLS->GetText();
					str >> temp;
					if(temp=="true")
					{
						performLocalSearch = true;
					}
					else
					{
						performLocalSearch = false;
					}
				}
				TiXmlElement* forbLSOp = paramALNS->FirstChildElement( "ForbiddenLSOperators" );
				if(forbLSOp)
				{
					TiXmlElement* firstOp = forbLSOp->FirstChildElement();
					while(firstOp)
					{
						addForbiddenLsOperator(firstOp->GetText());
						firstOp = firstOp->NextSiblingElement();
					}
				}
			}
		}
	}
}

void ALNS_Parameters::loadParameters(string path)
{
	ifstream comingFlow(path.data());
	string line;
	while(getline(comingFlow, line))
	{
		cout << "[DBG] " << line << endl;
		if(line.size() > 0 && line.at(0) != '#')
		{
			stringstream str;
			str << line;
			string id;
			string eq;
			str >> id;
			str >> eq;
			if(id == "maxNbIterations")
			{
				str >> maxNbIterations;
			}
			else if(id == "maxRunningTime")
			{
				str >> maxRunningTime;
			}
			else if(id == "maxNbIterationsNoImp")
			{
				str >> maxNbIterationsNoImp;
			}
			else if(id == "stoppingCriterion")
			{
				string temp;
				str >> temp;
				if(temp == "MAX_IT")
				{
					stopCrit = MAX_IT;
				}
				else if(temp == "MAX_RT")
				{
					stopCrit = MAX_RT;
				}
				else if(temp == "MAX_IT_NO_IMP")
				{
					stopCrit = MAX_IT_NO_IMP;
				}
				else if(temp == "ALL")
				{
					stopCrit = ALL;
				}
				else
				{
					assert(false);
				}
			}
			else if(id == "noise")
			{
				string temp;
				str >> temp;
				if(temp == "true")
				{
					noise = true;
				}
				else
				{
					noise = false;
				}
			}
			else if(id == "timeSegmentIt")
			{
				str >> timeSegmentsIt;
			}
			else if(id == "nbItBeforeReinit")
			{
				str >> nbItBeforeReinit;
			}
			else if(id == "sigma1")
			{
				str >> sigma1;
			}
			else if(id == "sigma2")
			{
				str >> sigma2;
			}
			else if(id == "sigma3")
			{
				str >> sigma3;
			}
			else if(id == "rho")
			{
				str >> rho;
			}
			else if(id == "minimumWeight")
			{
				str >> minimumWeight;
			}
			else if(id == "maximumWeight")
			{
				str >> maximumWeight;
			}
			else if(id == "probabilityOfNoise")
			{
				str >> probabilityOfNoise;
			}
			else if(id == "reloadFrequency")
			{
				str >> reloadFrequency;
			}
			else if(id == "acKind")
			{
				string temp;
				str >> temp;
				if(temp == "SA")
				{
					acKind = SA;
				}
				else
				{
					assert(false);
				}
			}
			else if(id == "acPath")
			{
				str >> acPath;
			}
			else if(id == "statsGlobPath")
			{
				str >> statsGlobPath;
			}
			else if(id == "statsOpPath")
			{
				str >> statsOpPath;
			}
			else if(id == "logFrequency")
			{
				str >> logFrequency;
			}
			else if(id == "performLocalSearch")
			{
				string temp;
				str >> temp;
				if(temp == "true")
				{
					performLocalSearch=true;
				}
				else
				{
					performLocalSearch=false;
				}
			}
			else if(id == "forbiddenOperator")
			{
				int posEq = line.find('=',0);
				string ss = line.substr(posEq+2,line.length());
				addForbiddenOperator(ss);
			}
			else if(id == "forbiddenLsOperator")
			{
				int posEq = line.find('=',0);
				string ss = line.substr(posEq+2,line.length());
				addForbiddenLsOperator(ss);
			}
			else
			{
				assert(false);
			}
		}
	}
}

void ALNS_Parameters::sanityChecks()
{

}

ALNS_Parameters::ALNS_Parameters(ALNS_Parameters& p)
{
	maxNbIterations = p.maxNbIterations;

	maxRunningTime = p.maxRunningTime;

	maxNbIterationsNoImp = p.maxNbIterationsNoImp;

	stopCrit = p.stopCrit;

	noise = p.noise;

	timeSegmentsIt = p.timeSegmentsIt;

	nbItBeforeReinit = p.nbItBeforeReinit;

	sigma1 = p.sigma1;

	sigma2 = p.sigma2;

	sigma3 = p.sigma3;

	rho = p.rho;

	minimumWeight = p.minimumWeight;

	maximumWeight = p.maximumWeight;

	acKind = p.acKind;

	acPath = p.acPath;

	logFrequency = p.logFrequency;

	performLocalSearch = p.performLocalSearch;

	minDestroyPerc = p.minDestroyPerc;

	maxDestroyPerc = p.maxDestroyPerc;

	lock = false;
}
